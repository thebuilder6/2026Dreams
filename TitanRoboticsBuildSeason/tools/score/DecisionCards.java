import java.io.IOException;
import java.nio.charset.StandardCharsets;
import java.nio.file.Files;
import java.nio.file.Path;
import java.nio.file.Paths;
import java.util.ArrayList;
import java.util.List;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

import frc.robot.Sim.HubSchedule;
import frc.robot.Utils.AllianceFlipUtil;
import frc.robot.Intelligence.AIActionIntent;
import frc.robot.Intelligence.Archetype;
import frc.robot.Intelligence.JevDecisionEngine;
import frc.robot.Intelligence.ClairvoyantKnowledge;
import frc.robot.Intelligence.MatchKnowledge;
import frc.robot.Intelligence.ObservedKnowledge;
import frc.robot.Intelligence.WorldState;

/**
 * Decision cards: a hand-reviewable probe of the Jev decision layer.
 *
 * <p>Why this exists. The headless 3v3 cannot currently serve as a fitness
 * function — see docs/SCORE_RIG_RESULTS.md. Its variance is dominated by two
 * causes we do not control (unseeded MapleSim physics, and a bimodal "collapse"
 * where the bots choose to hold and score nothing). A 150 s match therefore
 * cannot resolve whether a policy weight changed anything.
 *
 * <p>But {@code JevDecisionEngine.evaluatePolicy} is a *pure function* of
 * (WorldState, MatchKnowledge, Archetype). That makes the decision layer
 * directly measurable with no physics, no seeds, no match, and no variance at
 * all. This tool exercises that layer on hand-written situations so a human can
 * judge whether each decision is the right one.
 *
 * <p>The main diagnostic is a <b>held-fuel sweep</b>: for each situation the
 * engine is re-evaluated at every held-fuel count from 0 to 30, and the winning
 * objective is printed per row. That surfaces threshold behaviour and priority
 * inversions directly — a boundary at 8 or 16 shows up as a visible step, and
 * an inversion shows up as two objectives trading places in a way that looks
 * backwards. You do not need to already know the bug exists to see it.
 *
 * <p><b>Every card is evaluated once per alliance.</b> The Red pass mirrors the
 * geometry through {@code AllianceFlipUtil} and flips the shift seed, giving a true
 * mirror of the same situation: same phase, same relative hub roles. A card whose two
 * passes choose different objectives is flagged <b>ALLIANCE ASYMMETRY</b>, which
 * points at a hardcoded Blue assumption, a zone test, or a hub lookup in the utility
 * matrix. This codebase has shipped two such bugs -- {@code WorldStateBuilder} once
 * passed the human player's hub into a sim bot's opponent slot (inverted for half of
 * every match, for every bot), and the first version of these cards had the
 * SHIFT1/SHIFT2 polarity backwards. Neither was visible from a single alliance.
 *
 * <p>The subtlety worth knowing: flipping the alliance inside a <i>fixed</i> phase is
 * <b>not</b> a mirror. The shift seed decides which alliance sits out, so with the
 * default seed {@code 'R'} Blue is live in SHIFT1 and Red is live in SHIFT2. Swapping
 * the hub flags alone therefore compares SHIFT1 against SHIFT2 and reports differences
 * that are correct behaviour. Both the seed and the flags must flip together.
 *
 * <p>Usage:
 * <pre>
 *   powershell -File tools\score\run-cards.ps1
 *   powershell -File tools\score\run-cards.ps1 -TsV path\to\cards.tsv -Out path\to\out.md
 * </pre>
 *
 * <p>The first run writes a fully-populated template TSV with the card
 * situations filled in and the {@code expected} column blank. Fill in
 * {@code expected} with the objective you think is correct, re-run, and the
 * report marks each card PASS / MISMATCH / UNREVIEWED.
 *
 * <p>Convention: every card is evaluated as the <b>Blue</b> alliance
 * ({@code isRedAlliance=false}) with Blue-origin field coordinates, per the
 * repo-wide Blue-only coordinate rule. "Hub" therefore always means the Blue
 * hub at (4.6256, 4.0346). Chassis velocity is zero on every card, so
 * settle-gate behaviour is not exercised — these cards test objective choice,
 * not the shoot gate.
 *
 * <p>Rows with {@code S<seed>r<replica>-<label>-<t>s} ids are snapshots sampled
 * from a live sim match by {@code Sim/CardSnapshotSampler} (opt-in
 * {@code -PsnapshotCards}), not hand-authored situations. Their
 * {@code situation} text records the provenance and the situated objective;
 * {@code expected} is blank until a human reviews them.
 *
 * <p>This is a development tool. It is not robot code and is not on the roboRIO
 * path.
 */
public final class DecisionCards {

    // FieldMap.Hubs.BLUE_HUB_2D — used only to phrase distances in the report.
    private static final double BLUE_HUB_X = 4.6256;
    private static final double BLUE_HUB_Y = 4.0346;

    /**
     * TSV column order. The 19 base columns (id through notes) match the
     * on-disk {@code decision_cards.tsv}. Eight optional columns are appended
     * after {@code notes} so pre-existing 19-col rows still parse, defaulting
     * to honest zeros / default hardware / legacy rosters:
     * allianceZoneFuel, midfieldFuel, opponentZoneFuel (global zone fuel fed
     * to clairvoyant knowledge), hasShooter, ballCapacity, hasClimber
     * (WorldState hardware capabilities), allyPoses, oppExtras (explicit
     * roster pose lists).
     *
     * <p>Roster lists need no new columns: ally poses derive from
     * {@code alliesHeldFuel} and opponent poses from {@code oppObserved} plus
     * {@code oppZonePieces}. The report prints the resulting lists per card.
     * Two optional explicit-pose columns ({@code allyPoses}, {@code oppExtras},
     * {@code "x,y;x,y"} Blue-origin) override that derivation when non-blank.
     */
    private static final String[] COLUMNS = {
            "id", "archetype", "selfX", "selfY", "heldFuel", "matchTime",
            "hubActive", "oppHubActive", "timeToShift",
            "oppX", "oppY", "auto", "oppObserved", "scoreDiff", "oppZonePieces", "alliesHeldFuel",
            "situation", "expected", "notes",
            "allianceZoneFuel", "midfieldFuel", "opponentZoneFuel",
            "hasShooter", "ballCapacity", "hasClimber",
            "allyPoses", "oppExtras"
    };

    /** One evaluatable situation plus the human's answer. */
    private record Card(
            String id,
            Archetype archetype,
            double selfX, double selfY,
            int heldFuel,
            double matchTime,
            boolean hubActive,
            boolean oppHubActive,
            double timeToShift,
            boolean hubActiveAfter,
            boolean oppHubActiveAfter,
            double oppX, double oppY,
            boolean auto,
            boolean oppObserved,
            int scoreDiff,
            int oppZonePieces,
            int alliesHeldFuel,
            String situation,
            String expected,
            String notes,
            // Global zone fuel, fed to clairvoyant knowledge. `oppZonePieces`
            // above is only how many synthetic opponent-zone *poses* are
            // fabricated; these are the counts the harvest objectives actually
            // consume. Default 0 so pre-existing 19-col rows evaluate with an
            // empty field, matching the old arena-read fallback.
            int allianceZoneFuel,
            int midfieldFuel,
            int opponentZoneFuel,
            // WorldState hardware capabilities (added to WorldState 2026-10-07).
            // Defaults match WorldState.DEFAULT_* so old rows are unaffected.
            boolean hasShooter,
            int ballCapacity,
            boolean hasClimber,
            // Explicit roster pose lists, Blue-origin "x,y;x,y" (mirrored for
            // the Red pass like every other pose column). Blank means legacy
            // derivation: one ally at (3,2) iff alliesHeldFuel > 0, and
            // oppZonePieces synthetics from (12.0, 2.0). Non-blank replaces
            // that derivation exactly, so a card can place two allies, put
            // extras somewhere other than the synthetic line, or assert an
            // empty roster while holding fuel. The tracked opponent
            // (oppX, oppY iff oppObserved) is unaffected: it stays the
            // vision-tracked mark that derives opponentObserved.
            String allyPoses,
            String oppExtras) {}

    private DecisionCards() {}

    public static void main(String[] args) throws IOException {
        HAL.initialize(500, 0);

        Path tsv = Paths.get(args.length > 0 ? args[0] : "tools/score/decision_cards.tsv");
        Path out = Paths.get(args.length > 1 ? args[1] : "results/decision_cards.md");

        if (!Files.exists(tsv)) {
            writeTemplate(tsv);
            System.out.println("[cards] wrote template -> " + tsv);
            System.out.println("[cards] fill in the 'expected' column, then re-run.");
        }

        List<Card> cards = readTsv(tsv);
        if (cards.isEmpty()) {
            System.out.println("[cards] no cards in " + tsv);
            return;
        }

        StringBuilder md = new StringBuilder();
        md.append("# Jev Decision Cards\n\n");
        md.append("Generated by `tools/score/DecisionCards.java` — do not hand-edit this file.\n");
        md.append("Edit the `expected` column in `tools/score/decision_cards.tsv` and re-run.\n\n");
        md.append("- Cards: **").append(cards.size()).append("**\n");
        md.append("- Blue alliance, Blue-origin coordinates, zero chassis velocity\n\n");

        int pass = 0, mismatch = 0, unreviewed = 0;
        List<String> mismatches = new ArrayList<>();

        md.append("## Summary\n\n");
        md.append("Each card runs three times: as Blue clairvoyant, as mirrored Red clairvoyant, ")
          .append("and under the **observed** tier (what a real robot can honestly sense). ")
          .append("The verdict is judged against the Blue clairvoyant run.\n\n");
        md.append("| id | phase | next shift | archetype | expected | as Blue | as Red | observed | verdict |\n");
        md.append("|---|---|---|---|---|---|---|---|---|\n");

        int inconsistent = 0;
        int disagree = 0;
        int tierDiff = 0;
        for (Card c : cards) {
            boolean twoSets = c.archetype() != Archetype.CO_PILOT;
            AIActionIntent blue = evaluate(c, c.heldFuel(), false);
            String blueName = blue.objective().name();
            AIActionIntent red = twoSets ? evaluate(c, c.heldFuel(), true) : null;
            String redName = red == null ? "_(single)_" : red.objective().name();
            // The observed tier always runs, for every archetype. CO_PILOT is
            // excluded from the alliance mirror but not from this: the whole
            // point is "would a real robot, with no opponent tracker and no
            // field-fuel sensor, actually do this?"
            AIActionIntent observed = evaluate(c, c.heldFuel(), false, true);
            String obsName = observed.objective().name();
            String actual = blueName;

            String verdict;
            if (c.expected() == null || c.expected().isBlank()) {
                verdict = "UNREVIEWED";
                unreviewed++;
            } else if (c.expected().trim().equalsIgnoreCase(blueName)) {
                verdict = "PASS";
                pass++;
            } else {
                verdict = "**MISMATCH**";
                mismatch++;
                mismatches.add(c.id() + ": expected " + c.expected().trim()
                        + ", engine chose " + blueName);
            }
            if (twoSets && !blueName.equals(redName)) {
                verdict += " **ALLIANCE ASYMMETRY**";
                disagree++;
                mismatches.add(c.id() + ": Blue chose " + blueName
                        + " but Red chose " + redName + " for the mirrored state");
            }
            if (!blueName.equals(obsName)) {
                verdict += " **TIER DIFFERS**";
                tierDiff++;
                mismatches.add(c.id() + ": clairvoyant chose " + blueName
                        + " but the observed tier chose " + obsName
                        + " -- a real robot cannot do the clairvoyant answer");
            }
            List<String> issues = consistencyIssues(c);
            if (!issues.isEmpty()) {
                inconsistent++;
                verdict = "**INVALID STATE**";
            }
            md.append("| `").append(c.id()).append("` | ")
                    .append(HubSchedule.phaseFor(c.matchTime(), c.auto()))
                    .append(" | ").append(describeTransition(c))
                    .append(" | ").append(c.archetype())
                    .append(" | ").append(c.expected() == null || c.expected().isBlank()
                            ? "_(blank)_" : c.expected().trim())
                    .append(" | `").append(blueName).append("`")
                    .append(" | ").append(redName.startsWith("_") ? redName : "`" + redName + "`")
                    .append(" | `").append(obsName).append("`")
                    .append(" | ").append(verdict).append(" |\n");
        }

        if (tierDiff > 0) {
            md.append("\n> **").append(tierDiff).append(" card(s) choose a DIFFERENT objective")
                    .append(" under the observed tier** — the same card, the same clock, the same")
                    .append(" geometry, but a robot that has no opponent tracker and no field-fuel")
                    .append(" sensor. The clairvoyant answer is unreachable on real hardware for")
                    .append(" these cards. This is the measurement `docs/KNOWLEDGE_MODEL.md` asks")
                    .append(" for: it shows which objectives a defender actually retains once the")
                    .append(" clairvoyant tier stops feeding it poses and zone-fuel counts.\n");
        }

        if (disagree > 0) {
            md.append("\n> **").append(disagree).append(" card(s) choose a DIFFERENT objective")
                    .append(" depending on which alliance is asking.** The card is authored in")
                    .append(" Blue-origin coordinates with hub flags written as mine/theirs, so")
                    .append(" the Red pass is the same situation mirrored. A disagreement means")
                    .append(" something in the utility matrix is not alliance-symmetric — a")
                    .append(" hardcoded Blue assumption, a zone test, or a hub lookup. This is")
                    .append(" exactly how `WorldStateBuilder` shipped an inverted opponent hub")
                    .append(" for half of every match, and how these cards first shipped with the")
                    .append(" SHIFT1/SHIFT2 polarity backwards.\n");
        }

        if (inconsistent > 0) {
            md.append("\n> **").append(inconsistent).append(" card(s) describe a state the robot cannot")
                    .append(" reach** — the clock and the hub flags disagree. Those rows are marked")
                    .append(" INVALID STATE and their answers are meaningless until the TSV is fixed.")
                    .append(" See the per-card detail for the specific contradiction.\n");
        }

        md.append("\n**").append(pass).append(" pass, ").append(mismatch)
                .append(" mismatch, ").append(unreviewed).append(" unreviewed.**\n");

        if (!mismatches.isEmpty()) {
            md.append("\n### Mismatches\n\n");
            for (String s : mismatches) md.append("- ").append(s).append('\n');
        }

        md.append("\n---\n\n## Geometry probe — where is \"in shooting range\"?\n\n");
        md.append("`JevDecisionEngine:352` sets `inShootingRange = distToSelfHub <= 4.0`, and that flag\n");
        md.append("collapses `minFuelToScore` to **1** (`:354`) instead of 4 or 16. The table below\n");
        md.append("walks a Blue bot along y = 4.03 m (the hub centreline) and prints the distance.\n\n");
        md.append("```\n");
        md.append(String.format("%-9s %-14s %-12s %s%n", "x (m)", "dist to hub", "in range?", "in Blue zone?"));
        for (double x : new double[] {0.0, 0.5, 1.0, 2.0, 3.0, 4.0, 4.6256, 5.5, 6.5, 7.5, 8.2705, 9.0, 10.0}) {
            double d = Math.abs(x - BLUE_HUB_X);
            md.append(String.format("%-9.2f %-14.2f %-12s %s%n", x, d,
                    d <= 4.0 ? "YES" : "no", x <= 4.6256 ? "yes" : "no"));
        }
        md.append("```\n\n");
        md.append("**Read this before judging any card.** A 4.0 m radius centred on the hub covers the\n");
        md.append("entire home alliance zone *and* the whole Blue half of the field out to the\n");
        md.append("centerline. A Blue bot is therefore \"in shooting range\" almost everywhere it can\n");
        md.append("legally be, which means:\n\n");
        md.append("- `minFuelToScore` is **1** for essentially every reachable Blue position. The\n");
        md.append("  1 / 4 / 16 batch ladder only engages past x ~ 8.6 m, i.e. after crossing the\n");
        md.append("  centerline into the opponent's half.\n");
        md.append("- `CYCLE_SCORE_HUB`'s `loadRatio` is pinned to 1.0, so its utility is 0.98 and it\n");
        md.append("  outranks VACUUM (max 0.98) and SWEEP (0.96) whenever the hub is active and the\n");
        md.append("  bot holds **any** fuel.\n\n");
        md.append("Which means the engine's effective policy in its own half is: *score if you hold\n");
        md.append("anything, harvest if you are empty*. There is no distinct \"stage and wait for the\n");
        md.append("shift\" behaviour on an active hub. Whether that is the intended strategy is one of\n");
        md.append("the things these cards are for — see D04/D05, which probe it from outside the zone.\n\n");

        md.append("---\n\n## Cards\n");

        for (Card c : cards) {
            md.append("\n### `").append(c.id()).append("` — ").append(c.archetype()).append("\n\n");
            md.append("**Situation.** ").append(c.situation()).append("\n\n");
            if (c.notes() != null && !c.notes().isBlank()) {
                md.append("**Why it matters.** ").append(c.notes()).append("\n\n");
            }
            md.append("```\n");
            md.append(String.format("self            (%.2f, %.2f)   %.1f m from Blue hub%n",
                    c.selfX(), c.selfY(), dist(c.selfX(), c.selfY())));
            md.append(String.format("opponent        (%.2f, %.2f)%n", c.oppX(), c.oppY()));
            md.append(String.format("held fuel       %d      match time left %.0f s      AUTO %s%n",
                    c.heldFuel(), c.matchTime(), c.auto()));
            md.append(String.format("our hub         %s      opp hub %s      shift in %.1f s%n",
                    c.hubActive() ? "ACTIVE" : "inactive",
                    c.oppHubActive() ? "ACTIVE" : "inactive", c.timeToShift()));
            md.append(String.format("next shift      ours %-8s  theirs %-8s  %s%n",
                    c.hubActiveAfter() ? "ACTIVE" : "inactive",
                    c.oppHubActiveAfter() ? "ACTIVE" : "inactive",
                    describeTransition(c)));
            md.append(String.format("observed opp %-5s  score diff %+d  opp-zone pieces %d  allies holding %d%n",
                    c.oppObserved(), c.scoreDiff(), c.oppZonePieces(), c.alliesHeldFuel()));
            md.append(String.format("zone fuel       alliance %d  midfield %d  opponent %d  (clairvoyant; observed tier sees 0/0/0)%n",
                    c.allianceZoneFuel(), c.midfieldFuel(), c.opponentZoneFuel()));
            md.append(String.format("rosters         ally %s  opponent %s%n",
                    allyRosterSummary(c), rosterSummary(c)));
            md.append(String.format("hardware        shooter %s  capacity %d  climber %s%n",
                    c.hasShooter() ? "yes" : "NO",
                    c.ballCapacity(),
                    c.hasClimber() ? "yes" : "no"));
            md.append("```\n\n");

            List<String> issues = consistencyIssues(c);
            if (!issues.isEmpty()) {
                md.append("> **INVALID STATE — this card is not reachable in a real match:**\n");
                for (String s : issues) md.append("> - ").append(s).append('\n');
                md.append(">\n> The answer below still describes what the engine does with these\n");
                md.append("> numbers, but the situation itself cannot occur, so do not judge the\n");
                md.append("> policy on it. Fix the TSV clock/hub columns.\n\n");
            }

            md.append("Phase at ").append(fmt(c.matchTime())).append(" s remaining: **")
                    .append(HubSchedule.phaseFor(c.matchTime(), c.auto())).append("**\n\n");

            boolean twoSets = c.archetype() != Archetype.CO_PILOT;
            AIActionIntent nominal = evaluate(c, c.heldFuel(), false);
            AIActionIntent redNominal = twoSets ? evaluate(c, c.heldFuel(), true) : null;
            AIActionIntent obsNominal = evaluate(c, c.heldFuel(), false, true);
            md.append("**As Blue, clairvoyant, the engine chose `")
                    .append(nominal.objective().name()).append("`** — ")
                    .append(nominal.rationale()).append("\n\n");
            if (twoSets) {
                md.append("**As Red (mirrored), it chose `" + redNominal.objective().name() + "`** — ")
                        .append(redNominal.rationale()).append("\n\n");
                if (!nominal.objective().equals(redNominal.objective())) {
                    md.append("> **ALLIANCE ASYMMETRY** — the same situation, mirrored, produces a")
                            .append(" different objective. Something in the utility matrix is not")
                            .append(" alliance-symmetric.\n\n");
                }
                Pose2d redPose = AllianceFlipUtil.apply(
                        new Pose2d(c.selfX(), c.selfY(), new Rotation2d()), true);
                md.append("Red-view self pose (mirrored via `AllianceFlipUtil`): `(")
                        .append(String.format("%.2f, %.2f", redPose.getX(), redPose.getY()))
                        .append(")`, hub flags swapped to mine/theirs.\n\n");
            } else {
                md.append("_(CO_PILOT cards are evaluated as a single alliance set — it is the")
                        .append(" player-facing archetype, not a sparring bot. The observed tier")
                        .append(" still runs.)_\n\n");
            }
            md.append("**Under the observed tier** (`ObservedKnowledge.selfOnly()` — no opponent")
                    .append(" poses, no zone-fuel counts), it chose `")
                    .append(obsNominal.objective().name()).append("`** — ")
                    .append(obsNominal.rationale()).append("\n\n");
            if (!nominal.objective().equals(obsNominal.objective())) {
                md.append("> **TIER DIFFERS** — the clairvoyant answer above is **unreachable on a")
                        .append(" real robot**. The card reads the same way, but a bot with no")
                        .append(" opponent tracker and no field-fuel sensor picks something else.")
                        .append(" Per `docs/KNOWLEDGE_MODEL.md`, that diff is the measurement: it")
                        .append(" shows which objectives a defender actually retains on hardware.\n\n");
            }

            // One evaluation per axis, reused. Calling evaluate() per cell would
            // be correct but needlessly repeated: it re-derives the phase, the
            // seed, and the zone counts every time.
            String colBlue = String.format("%.2f", nominal.confidence());
            String colObs = String.format("%.2f", obsNominal.confidence());
            String navB = pose(nominal.navigationTarget());
            String navO = pose(obsNominal.navigationTarget());
            md.append("| detail | as Blue | as Red | observed |\n");
            md.append("|---|---|---|---|\n");
            md.append("| next objective | `").append(nominal.plan().nextObjective().name())
                    .append("` | `")
                    .append(redNominal == null ? "—"
                            : redNominal.plan().nextObjective().name())
                    .append("` | `").append(obsNominal.plan().nextObjective().name()).append("` |\n");
            md.append("| confidence | ").append(colBlue).append(" | ")
                    .append(redNominal == null ? "—" : String.format("%.2f", redNominal.confidence()))
                    .append(" | ").append(colObs).append(" |\n");
            md.append("| intake | ").append(nominal.intakeCommand()).append(" | ")
                    .append(redNominal == null ? "—" : redNominal.intakeCommand().toString())
                    .append(" | ").append(obsNominal.intakeCommand()).append(" |\n");
            md.append("| shooter | ").append(nominal.shooterCommand()).append(" | ")
                    .append(redNominal == null ? "—" : redNominal.shooterCommand().toString())
                    .append(" | ").append(obsNominal.shooterCommand()).append(" |\n");
            md.append("| trigger kicker | ").append(nominal.triggerFeedKicker()).append(" | ")
                    .append(redNominal == null ? "—" : String.valueOf(redNominal.triggerFeedKicker()))
                    .append(" | ").append(obsNominal.triggerFeedKicker()).append(" |\n");
            md.append("| nav target | `").append(navB).append("` | `")
                    .append(redNominal == null ? "—" : pose(redNominal.navigationTarget()))
                    .append("` | `").append(navO).append("` |\n");
            md.append("\n**Held-fuel sweep** — winning objective at every held-fuel count; ")
                    .append("nominal is marked `*`:\n\n```\n");
            md.append(twoSets ? sweepBoth(c) : sweep(c));
            md.append("```\n\n");
        }

        Files.createDirectories(out.getParent());
        Files.writeString(out, md.toString(), StandardCharsets.UTF_8);

        System.out.println("[cards] " + cards.size() + " card(s): "
                + pass + " pass, " + mismatch + " mismatch, " + unreviewed + " unreviewed");
        System.out.println("[cards] report -> " + out);
        for (String s : mismatches) System.out.println("  MISMATCH " + s);
    }

    /** Winning objective for held fuel 0..30, with the nominal count flagged. */
    private static String sweep(Card c) {
        return sweep(c, false, "");
    }

    /** Two-column sweep; '*' marks the nominal held count. */
    private static String sweepBoth(Card c) {
        return sweep(c, true, "as Red ");
    }

    private static String sweep(Card c, boolean withRed, String redPrefix) {
        StringBuilder sb = new StringBuilder();
        if (withRed) {
            sb.append(String.format("%-6s %-22s %-7s %s%n", "held", "as Blue", "", "as Red"));
        }
        String prevB = null, prevR = null;
        int runStart = 0;
        for (int held = 0; held <= 30; held++) {
            String b = evaluate(c, held, false).objective().name();
            String r = withRed ? evaluate(c, held, true).objective().name() : null;
            if (prevB == null) {
                prevB = b; prevR = r; runStart = held;
            } else if (!b.equals(prevB) || (withRed && !r.equals(prevR))) {
                appendRun(sb, prevB, runStart, held - 1, c.heldFuel(), prevR, withRed, redPrefix);
                prevB = b; prevR = r; runStart = held;
            }
        }
        appendRun(sb, prevB, runStart, 30, c.heldFuel(), prevR, withRed, redPrefix);
        return sb.toString();
    }

    private static void appendRun(StringBuilder sb, String obj, int from, int to, int nominal,
            String redObj, boolean withRed, String redPrefix) {
        String range = (from == to) ? String.valueOf(from) : (from + "-" + to);
        boolean hit = nominal >= from && nominal <= to;
        if (withRed) {
            String flag = (redObj != null && !redObj.equals(obj)) ? "  <-- ASYMMETRY" : "";
            sb.append(String.format("%-6s %-22s %-7s %s%s%n", range, obj,
                    hit ? "*" : "", redPrefix + (redObj == null ? "-" : redObj), flag));
        } else {
            sb.append(String.format("%-6s %-22s %s%n", range, obj, hit ? "*" : ""));
        }
    }

    private static AIActionIntent evaluate(Card c, int heldFuel) {
        return evaluate(c, heldFuel, false);
    }

    /**
     * Evaluates a card from one alliance's point of view.
     *
     * <p>Cards are authored in Blue-origin coordinates with hub flags written
     * alliance-relatively ("mine" / "theirs"). For the Red pass the geometry is
     * mirrored through {@link AllianceFlipUtil} — the single owner, per the
     * repo-wide Blue-only coordinate rule — and the two hub flags swap, because
     * {@code isAllianceHubActive} means "mine" whichever alliance that is.
     *
     * <p>Running both passes is not belt-and-braces. This codebase has shipped
     * two alliance-asymmetry bugs: {@code WorldStateBuilder} once passed the
     * human player's hub into a sim bot's opponent slot (inverted for half the
     * match, for every bot), and the first version of these cards had the
     * SHIFT1/SHIFT2 polarity backwards. Neither is visible from a single set.
     */
    private static AIActionIntent evaluate(Card c, int heldFuel, boolean asRed) {
        return evaluate(c, heldFuel, asRed, false);
    }

    /**
     * Evaluates a card under one (alliance, knowledge-tier) combination.
     *
     * <p>{@code observedTier} selects the knowledge record. {@code false} is the
     * clairvoyant sim-sparring tier, which sees every pose and all three zone-fuel
     * counts. {@code true} is {@link ObservedKnowledge}: what a real robot can
     * honestly produce &mdash; no opponent tracker, no field-fuel sensor, so the
     * zone counts are zero and other robots are simply absent.
     */
    private static AIActionIntent evaluate(Card c, int heldFuel, boolean asRed, boolean observedTier) {
        Pose2d self = new Pose2d(c.selfX(), c.selfY(), new Rotation2d(Math.toRadians(180)));
        Pose2d opp = new Pose2d(c.oppX(), c.oppY(), new Rotation2d());
        if (asRed) {
            self = AllianceFlipUtil.apply(self, true);
            opp = AllianceFlipUtil.apply(opp, true);
        }
        ChassisSpeeds still = new ChassisSpeeds();

        // Hub state is DERIVED per pass from (phase, seed), not read from the card
        // and not swapped. This is the subtle part: flipping the alliance inside a
        // FIXED phase does not mirror the situation, because the shift seed decides
        // which alliance sits out. With the default seed 'R', Blue is live in SHIFT1
        // and Red is live in SHIFT2 -- so a naive swap compares SHIFT1 against
        // SHIFT2 and reports a difference that is correct behaviour, not a bug. The
        // Red pass therefore also FLIPS THE SEED, which is what makes it a true
        // mirror: same phase, same relative hub roles, mirrored geometry. Any
        // remaining disagreement is genuine non-symmetry in the utility matrix.
        HubSchedule.Phase phase = HubSchedule.phaseFor(c.matchTime(), c.auto());
        char seed = asRed ? 'B' : 'R';
        boolean mine = HubSchedule.isHubActive(asRed, phase, seed);
        boolean theirs = HubSchedule.isHubActive(!asRed, phase, seed);
        boolean mineAfter = HubSchedule.isHubActive(asRed, HubSchedule.nextPhase(phase), seed);
        boolean theirsAfter = HubSchedule.isHubActive(!asRed, HubSchedule.nextPhase(phase), seed);

        // The tracked opponent is the card's own (oppX, oppY) and exists IFF
        // `oppObserved` is set. This is load-bearing, not decoration: since the
        // clairvoyant/observed split, `ClairvoyantKnowledge.opponentObserved()`
        // is DERIVED from `!opponentPoses.isEmpty()` rather than passed as a
        // flag, deliberately, so that a record claiming to see an opponent
        // while carrying no poses becomes inexpressible. Driving the pose list
        // off a different column therefore does not merely look wrong -- it
        // silently zeroes lane denial, shadow and intercept via the tier-1 gate
        // at JevDecisionEngine:552 for every card that set oppObserved=true.
        // The old wiring fed opponentPoses from `oppZonePieces`, so a card
        // saying "opponent observed" with zero opponent-zone pieces claimed
        // vision it did not have, and every defensive card quietly measured the
        // no-opponent path instead.
        List<Pose2d> oppPoses = new ArrayList<>();
        if (c.oppObserved()) {
            oppPoses.add(opp);   // already mirrored for the Red pass
        }
        // Additional synthetic opponents in the opponent zone. These exist so
        // zone-scoring objectives are reachable at all, not to represent vision.
        // An explicit oppExtras list replaces this synthesis exactly.
        if (!c.oppExtras().isBlank()) {
            for (Pose2d p : parsePoseList(c.oppExtras(), false, "oppExtras", c.id())) {
                oppPoses.add(asRed ? AllianceFlipUtil.apply(p, true) : p);
            }
        } else {
            for (int i = 0; i < c.oppZonePieces(); i++) {
                Pose2d p = new Pose2d(12.0 + 0.4 * i, 2.0 + 0.6 * i, new Rotation2d());
                oppPoses.add(asRed ? AllianceFlipUtil.apply(p, true) : p);
            }
        }
        List<Pose2d> allyPoses;
        if (!c.allyPoses().isBlank()) {
            allyPoses = parsePoseList(c.allyPoses(), asRed, "allyPoses", c.id());
        } else if (c.alliesHeldFuel() > 0) {
            Pose2d a = new Pose2d(3.0, 2.0, new Rotation2d());
            allyPoses = List.of(asRed ? AllianceFlipUtil.apply(a, true) : a);
        } else {
            allyPoses = List.of();
        }

        WorldState world = new WorldState(
                self, still, heldFuel,
                opp, still, c.matchTime(),
                mine, theirs, c.timeToShift(),
                /* isRedAlliance */ asRed, c.auto(),
                mineAfter, theirsAfter,
                c.hasShooter(), c.ballCapacity(), c.hasClimber());

        // Knowledge tier is a real axis, not a comment. A card evaluated only as
        // clairvoyant can look correct while the objective it chose is unreachable
        // for a robot that has no opponent tracker and no field-fuel sensor. Each
        // card therefore runs under BOTH tiers, and the verdict reports which tier
        // it holds for. The diff between the two is the measurement: it shows which
        // objectives a defender actually retains on real hardware.
        //
        // `oppObserved` on the card is honoured by the CLAIRVOYANT run only. The
        // observed run ignores it, because a real robot has no tracker regardless
        // of what a card claims -- that is the entire point of the tier.
        MatchKnowledge knowledge;
        if (observedTier) {
            // A real robot: self pose and hopper only. No opponent poses, no
            // zone-fuel counts (both are structurally absent, not zero-guessed).
            knowledge = ObservedKnowledge.selfOnly();
        } else {
            knowledge = new ClairvoyantKnowledge(
                    c.scoreDiff(), c.alliesHeldFuel(), 0, 0, 0,
                    allyPoses, oppPoses, List.of(), List.of(),
                    c.allianceZoneFuel(), c.midfieldFuel(), c.opponentZoneFuel());
        }

        // cloudContext null, blockedFuel empty, commitment null => the raw
        // uncoupled utility race. That is what we want to inspect; a commitment
        // latch would hide exactly the near-ties this tool exists to reveal.
        return JevDecisionEngine.getInstance()
                .evaluatePolicy(world, knowledge, c.archetype(), null, java.util.Set.of(), null);
    }

    private static double dist(double x, double y) {
        return Math.hypot(x - BLUE_HUB_X, y - BLUE_HUB_Y);
    }

    /** Formats a nav target as a coordinate pair, or a dash when there is none. */
    private static String pose(Pose2d p) {
        return p == null ? "—" : String.format("%.2f, %.2f", p.getX(), p.getY());
    }

    /**
     * Parses a Blue-origin pose list of the form {@code "x,y;x,y"} (whitespace
     * tolerant, rotation always zero). A blank spec yields an empty list; the
     * caller decides the legacy-derivation fallback. Mirroring is applied by
     * the caller per pose so explicit and legacy paths share one rule.
     */
    private static List<Pose2d> parsePoseList(String spec, boolean asRed, String column, String cardId) {
        List<Pose2d> out = new ArrayList<>();
        if (spec == null || spec.isBlank()) {
            return out;
        }
        for (String pair : spec.split(";", -1)) {
            if (pair.isBlank()) continue;
            String[] xy = pair.split(",", -1);
            if (xy.length != 2) {
                throw new IllegalArgumentException(
                        "card " + cardId + " column " + column + ": expected \"x,y\" but got \"" + pair.trim() + "\"");
            }
            Pose2d p = new Pose2d(Double.parseDouble(xy[0].trim()), Double.parseDouble(xy[1].trim()),
                    new Rotation2d());
            out.add(asRed ? AllianceFlipUtil.apply(p, true) : p);
        }
        return out;
    }

    /**
     * One-line roster summary matching the poses {@link #evaluate} synthesizes:
     * the tracked opponent at (oppX, oppY) iff oppObserved, plus the extras
     * (explicit oppExtras list, else oppZonePieces synthetics from (12.0, 2.0)).
     * Keeps the report honest about what clairvoyant knowledge actually carried.
     */
    private static String rosterSummary(Card c) {
        StringBuilder sb = new StringBuilder();
        if (c.oppObserved()) {
            sb.append(String.format("tracked at (%.2f, %.2f)", c.oppX(), c.oppY()));
        } else {
            sb.append("tracked none");
        }
        if (!c.oppExtras().isBlank()) {
            sb.append(" + explicit [").append(c.oppExtras().trim()).append("]");
        } else if (c.oppZonePieces() > 0) {
            sb.append(String.format(" + %d synthetic from (12.00, 2.00)", c.oppZonePieces()));
        }
        return sb.toString();
    }

    /**
     * One-line ally roster summary: the explicit allyPoses list when present,
     * else the legacy derivation (one ally at (3,2) iff alliesHeldFuel > 0).
     */
    private static String allyRosterSummary(Card c) {
        if (!c.allyPoses().isBlank()) {
            return "explicit [" + c.allyPoses().trim() + "]";
        }
        return c.alliesHeldFuel() > 0 ? "1 at (3.00, 2.00)" : "none";
    }

    // ------------------------------------------------------------------
    // Phase consistency
    //
    // A card's (matchTime, auto, hubActive, oppHubActive, timeToShift) must be
    // mutually consistent with the official hub schedule, or it describes a state
    // the robot can never reach and the answer is meaningless. These checks are
    // derived from HubSchedule itself rather than a hand-copied phase table, so
    // they cannot drift if the schedule changes.
    // ------------------------------------------------------------------

    /** Match time remaining at which the current shift ends, by phase. */
    private static double shiftEnd(HubSchedule.Phase p) {
        return switch (p) {
            case SHIFT1 -> 105.0;
            case SHIFT2 -> 80.0;
            case SHIFT3 -> 55.0;
            case SHIFT4 -> 30.0;
            default -> Double.NaN;   // AUTO / TRANSITION / ENDGAME / DONE
        };
    }

    private static List<String> consistencyIssues(Card c) {
        List<String> out = new ArrayList<>();
        HubSchedule.Phase phase = HubSchedule.phaseFor(c.matchTime(), c.auto());
        // Default seed is 'R' (Red sits out first); see HubSchedule.getShiftSeed().
        boolean oursActive = HubSchedule.isHubActive(false, phase, 'R');
        boolean theirsActive = HubSchedule.isHubActive(true, phase, 'R');
        double end = shiftEnd(phase);

        if (oursActive != c.hubActive()) {
            out.add("our hub " + (c.hubActive() ? "ACTIVE" : "inactive")
                    + " but phase " + phase + " requires " + (oursActive ? "ACTIVE" : "inactive"));
        }
        if (theirsActive != c.oppHubActive()) {
            out.add("opponent hub " + (c.oppHubActive() ? "ACTIVE" : "inactive")
                    + " but phase " + phase + " requires " + (theirsActive ? "ACTIVE" : "inactive"));
        }
        // The after-shift flags are derived from phase + seed, so they are correct
        // by construction. Assert the derivation rather than trusting it.
        if (c.hubActiveAfter() != HubSchedule.isHubActive(false,
                HubSchedule.nextPhase(phase), 'R')
                || c.oppHubActiveAfter() != HubSchedule.isHubActive(true,
                        HubSchedule.nextPhase(phase), 'R')) {
            out.add("after-shift hub flags disagree with HubSchedule.nextPhase("
                    + phase + ")");
        }
        if (Double.isNaN(end)) {
            if (c.timeToShift() > 0.0) {
                out.add("timeToShift=" + c.timeToShift()
                        + " but phase " + phase + " has no shift countdown (must be 0)");
            }
        } else {
            double expected = Math.max(0.0, c.matchTime() - end);
            if (Math.abs(expected - c.timeToShift()) > 0.6) {
                out.add("timeToShift=" + c.timeToShift() + " but phase " + phase
                        + " at " + fmt(c.matchTime()) + " s remaining implies " + fmt(expected));
            }
        }
        return out;
    }

    private static String fmt(double v) {
        return (v == Math.rint(v)) ? String.valueOf((long) v) : String.format("%.1f", v);
    }

    /**
     * Plain-English description of what the next flip does to this robot.
     *
     * <p>Only two quadrants are reachable: ours-active-going-dark (SHIFT1/SHIFT3)
     * and ours-dark-about-to-open (SHIFT2/SHIFT4). AUTO/TRANSITION/ENDGAME have
     * both hubs live with no flip pending, so they are "no change". Both hubs are
     * never dark at the same time, so the "both dark" quadrant does not exist.
     */
    private static String describeTransition(Card c) {
        boolean losing = c.hubActive() && !c.hubActiveAfter();
        boolean opening = !c.oppHubActive() && c.oppHubActiveAfter();
        if (losing && opening) return "LAST WINDOW (ours closes, theirs opens)";
        if (losing) return "ours closes";
        if (opening) return "ours opens";
        if (!c.hubActive() && c.hubActiveAfter()) return "ours opens";
        if (c.oppHubActive() && !c.oppHubActiveAfter()) return "theirs closes";
        return "no change (both hubs live)";
    }

    // ------------------------------------------------------------------
    // TSV I/O
    // ------------------------------------------------------------------

    private static List<Card> readTsv(Path tsv) throws IOException {
        List<Card> cards = new ArrayList<>();
        for (String line : Files.readAllLines(tsv, StandardCharsets.UTF_8)) {
            if (line.isBlank() || line.startsWith("#")) continue;
            String[] f = line.split("\t", -1);
            if (f.length > 0 && f[0].trim().equalsIgnoreCase("id")) continue;  // header
            // Minimum is 19 columns (id through notes). Eight optional columns
            // may follow notes: allianceZoneFuel, midfieldFuel,
            // opponentZoneFuel, hasShooter, ballCapacity, hasClimber,
            // allyPoses, oppExtras.
            // A 19-col row predates those inputs and gets honest defaults
            // (empty field, default hardware, legacy roster derivation),
            // matching the old behaviour.
            if (f.length < 19) {
                System.out.println("[cards] skipping short line (" + f.length + " cols): " + line);
                continue;
            }
            try {
                // hubActive / oppHubActive / timeToShift are kept as TSV columns
                // purely as a CROSS-CHECK, validated by consistencyIssues(). At
                // evaluation time hub state is derived from (phase, seed) per pass,
                // because that is the only way both alliance passes can be true
                // mirrors of each other. The after-shift flags are derived outright.
                // NOTE: auto is column 11 (f[11]), not f[13] (scoreDiff).
                HubSchedule.Phase phase = HubSchedule.phaseFor(d(f[5]), b(f[11]));
                // Validate explicit pose specs now so a typo skips its row with
                // a message instead of aborting the whole run mid-evaluation.
                parsePoseList(f.length > 25 ? f[25].trim() : "", false, "allyPoses", f[0].trim());
                parsePoseList(f.length > 26 ? f[26].trim() : "", false, "oppExtras", f[0].trim());
                cards.add(new Card(
                        f[0].trim(),
                        Archetype.valueOf(f[1].trim()),
                        d(f[2]), d(f[3]),
                        i(f[4]),
                        d(f[5]),
                        b(f[6]), b(f[7]),
                        d(f[8]),
                        HubSchedule.isHubActive(false, HubSchedule.nextPhase(phase), 'R'),
                        HubSchedule.isHubActive(true, HubSchedule.nextPhase(phase), 'R'),
                        d(f[9]), d(f[10]),
                        b(f[11]), b(f[12]),
                        i(f[13]), i(f[14]), i(f[15]),
                        f[16].trim(), f[17].trim(), f[18].trim(),
                        // Optional tail columns (indices 19-24). Guard each
                        // individually so a partially-extended row still parses.
                        f.length > 19 && !f[19].isBlank() ? i(f[19]) : 0,
                        f.length > 20 && !f[20].isBlank() ? i(f[20]) : 0,
                        f.length > 21 && !f[21].isBlank() ? i(f[21]) : 0,
                        f.length > 22 && !f[22].isBlank() ? b(f[22]) : true,
                        f.length > 23 && !f[23].isBlank() ? i(f[23]) : 30,
                        f.length > 24 && !f[24].isBlank() ? b(f[24]) : false,
                        f.length > 25 ? f[25].trim() : "",
                        f.length > 26 ? f[26].trim() : ""));
            } catch (IllegalArgumentException e) {
                System.out.println("[cards] skipping unparseable line '" + f[0] + "': " + e.getMessage());
            }
        }
        return cards;
    }

    private static int i(String s) { return Integer.parseInt(s.trim()); }
    private static double d(String s) { return Double.parseDouble(s.trim()); }
    private static boolean b(String s) {
        String t = s.trim().toLowerCase();
        return t.equals("true") || t.equals("yes") || t.equals("1");
    }

    private static void writeTemplate(Path tsv) throws IOException {
        Files.createDirectories(tsv.getParent());
        StringBuilder sb = new StringBuilder();
        sb.append("# Jev decision cards — hand-reviewable probe of the decision layer.\n");
        sb.append("# Generated once by tools/score/DecisionCards.java; edit freely after that.\n");
        sb.append("# Tab-separated. One card per line. Fill in the 'expected' column with the\n");
        sb.append("# objective you think is correct; the tool then reports PASS/MISMATCH.\n");
        sb.append("#\n");
        sb.append("# EACH CARD IS EVALUATED TWICE, once per alliance (CO_PILOT cards excepted —\n");
        sb.append("# it is the player-facing archetype, not a sparring bot). The Red pass mirrors\n");
        sb.append("# the geometry through AllianceFlipUtil and flips the shift seed, so it is a true\n");
        sb.append("# mirror: same phase, same relative hub roles. If the two passes choose\n");
        sb.append("# different objectives the card is flagged ALLIANCE ASYMMETRY. Note that simply\n");
        sb.append("# swapping the hub flags WITHOUT flipping the seed is NOT a mirror — the seed\n");
        sb.append("# decides which alliance sits out, so that compares SHIFT1 against SHIFT2.\n");
        sb.append("#\n");
        sb.append("# Poses are BLUE-origin metres, zero chassis velocity. Hub = Blue hub at\n");
        sb.append("# (4.6256, 4.0346). selfRot is not a column: the engine is given a fixed 180 deg\n");
        sb.append("# facing so the shoot gate is always satisfiable.\n");
        sb.append("#\n");
        sb.append("# hubActive / oppHubActive / timeToShift are CROSS-CHECKS, not inputs. Hub\n");
        sb.append("# state is derived from (matchTime, phase, seed) at evaluation time, because\n");
        sb.append("# that is the only way both alliance passes can be mirrors. The tool flags any\n");
        sb.append("# card whose authored values contradict the schedule as INVALID STATE.\n");
        sb.append("#\n");
        sb.append("# Column meanings beyond the obvious:\n");
        sb.append("#   matchTime     SECONDS REMAINING, not elapsed. 150 = match start, 0 = buzzer.\n");
        sb.append("#   timeToShift   seconds until OUR hub changes state. 0 = both hubs active\n");
        sb.append("#                 (no shift countdown), matching HubSchedule.timeUntilHubShift().\n");
        sb.append("#   oppZonePieces synthetic opponent poses in the opponent zone, so POACH and\n");
        sb.append("#                 zone-scoring objectives are reachable at all (0 = none).\n");
        sb.append("#   alliesHeldFuel feeds SCREEN_FOR_ALLY; >0 also places one ally at (3,2).\n");
        sb.append("#   allianceZoneFuel / midfieldFuel / opponentZoneFuel feed clairvoyant\n");
        sb.append("#                 knowledge (harvest objectives consume them). Default 0.\n");
        sb.append("#   hasShooter / ballCapacity / hasClimber feed WorldState hardware\n");
        sb.append("#                 gates. Defaults true / 30 / false (WorldState.DEFAULT_*).\n");
        sb.append("#   allyPoses     explicit ally roster as \"x,y;x,y\" Blue-origin (max 2).\n");
        sb.append("#                 Blank = legacy: one ally at (3,2) iff alliesHeldFuel > 0.\n");
        sb.append("#   oppExtras     explicit extra opponent poses as \"x,y;x,y\" Blue-origin.\n");
        sb.append("#                 Blank = legacy: oppZonePieces synthetics from (12.0, 2.0).\n");
        sb.append("#                 The tracked opponent (oppX,oppY iff oppObserved) is separate\n");
        sb.append("#                 and unaffected by either column.\n");
        sb.append("#   expected      YOUR ANSWER, judged against the BLUE pass. Blank = UNREVIEWED.\n");
        sb.append("#   notes         why this card matters (shown in the report).\n");
        sb.append(String.join("\t", COLUMNS)).append('\n');

        // (id, archetype, selfX, selfY, held, matchTime, hubActive, oppHubActive,
        //  timeToShift, oppX, oppY, auto, oppObserved, scoreDiff, oppZonePieces,
        //  alliesHeldFuel, situation, expected, notes)
        Object[][] rows = {
            // =================================================================
            // Card set rebuilt 2026-09-28 around what we now know, rather than
            // accumulated around what we guessed. Cards that probed disproven
            // premises are GONE, not merely relabelled:
            //   - POACH_OPPONENT_ZONE probes. Its utility is a flat 0.78 and is
            //     beaten in every reachable state (VACUUM >= 0.82, STOCKPILE
            //     >= 0.86, CYCLE 0.98, SWEEP 0.96), so it looks structurally
            //     unreachable. Probing it told us nothing.
            //   - "both hubs dark" cards. Unreachable: SHIFT1-4 have exactly one
            //     live hub, AUTO/TRANSITION/ENDGAME have both.
            //   - minFuelToScore 4/16 rungs. Unreachable: the harvest force at
            //     JevDecisionEngine:551 overrides the ladder with a literal 8.
            //
            // Only two hub quadrators are reachable, and they alternate:
            //   SHIFT1/SHIFT3  ours ACTIVE -> dark    ("LAST WINDOW", theirs opens)
            //   SHIFT2/SHIFT4  ours dark  -> ACTIVE  ("ours opens", theirs closes)
            // and both-live phases (AUTO / TRANSITION / ENDGAME) are "no change".
            // With the default seed 'R', Blue scores in SHIFT1/SHIFT3.
            // matchTime is SECONDS REMAINING: 150 = start, 0 = buzzer.
            //
            // Geometry that matters: inShootingRange is dist <= 4.0 m from OUR
            // hub, which covers the entire home zone and out past the centerline
            // (3.64 m at x = 8.27). A bot is out of range only behind x ~ 0.63
            // or past x ~ 8.63. So "far from hub" cards sit at those extremes.
            //
            // (id, archetype, selfX, selfY, held, matchTime, hubActive,
            //  oppHubActive, timeToShift, oppX, oppY, auto, oppObserved,
            //  scoreDiff, oppZonePieces, alliesHeldFuel, situation, expected, notes)

            // --- Z: canary ----------------------------------------------------
            // Deliberately invalid. matchTime 130 is SHIFT1, where OUR hub must be
            // live, but this row claims it is dark. The checker must flag it; if
            // Z99 ever reports PASS the validator has regressed.
            row("Z99", "AUTONOMOUS_CYCLER", 2.0, 4.03, 10, 130, false, true, 0.0,
                    12.0, 4.0, false, true, 0, 0, 0,
                    "CANARY. SHIFT1 clock but our hub declared dark with no countdown.",
                    "", "Must be reported INVALID STATE. If it is not, the checker is broken."),

            // --- A: the baseline, so deviations stand out ---------------------
            row("A1", "AUTONOMOUS_CYCLER", 2.0, 4.03, 0, 140, true, true, 0.0,
                    12.0, 4.0, false, true, 0, 0, 0,
                    "TRANSITION, both hubs live, EMPTY.",
                    "", "The floor of the policy: hub live but nothing to score, so harvest."),
            row("A2", "AUTONOMOUS_CYCLER", 2.0, 4.03, 8, 140, true, true, 0.0,
                    12.0, 4.0, false, true, 0, 0, 0,
                    "TRANSITION, both hubs live, 8 fuel.",
                    "", "The ratchet: at 8 fuel it commits to scoring, not topping up."),
            row("A3", "AUTONOMOUS_CYCLER", 2.0, 4.03, 30, 140, true, true, 0.0,
                    12.0, 4.0, false, true, 0, 0, 0,
                    "TRANSITION, both hubs live, hopper FULL (30).",
                    "", "isInventoryFull should suppress VACUUM/STOCKPILE/SWEEP outright."),
            row("A4", "AUTONOMOUS_CYCLER", 2.0, 4.03, 20, 20, true, true, 0.0,
                    12.0, 4.0, false, true, 0, 0, 0,
                    "ENDGAME, 20 s left, both hubs live, 20 fuel.",
                    "", "The other both-live phase, at the other end of the match."),
            row("A5", "AUTONOMOUS_CYCLER", 2.0, 4.03, 7, 150, true, true, 0.0,
                    12.0, 4.0, false, true, 0, 0, 0,
                    "AUTO, 7 fuel — one short of AUTO_BATCH_MIN_FUEL.",
                    "", "Probes the autonomous batch gate from below."),
            row("A6", "AUTONOMOUS_CYCLER", 2.0, 4.03, 8, 150, true, true, 0.0,
                    12.0, 4.0, false, true, 0, 0, 0,
                    "AUTO, exactly 8 fuel — the batch threshold.",
                    "", "Should flip to committing to score at exactly 8."),
            row("A7", "AUTONOMOUS_CYCLER", 3.0, 4.03, 8, 140, true, true, 0.0,
                    5.2, 4.03, false, true, 0, 0, 0,
                    "In range (1.6 m from hub), 8 fuel, but an opponent parked 0.6 m in the lane.",
                    "", "isShootingLaneBlocked: opponent within 1.60 m and under 25 deg of the\n"
                        + "hub bearing. Should suppress firing but not change the objective."),

            // --- B: the batch threshold (effective 8, not the coded 16) -------
            // All out-of-range cards sit at x = 0.40, which is 4.23 m from the hub.
            row("B1", "AUTONOMOUS_CYCLER", 0.40, 4.03, 7, 140, true, true, 0.0,
                    12.0, 4.0, false, true, 0, 0, 0,
                    "Out of the 4.0 m radius (4.23 m), hub live, 7 fuel.",
                    "", "minFuelToScore says 16 here. 7 must not justify a scoring trip."),
            row("B2", "AUTONOMOUS_CYCLER", 0.40, 4.03, 8, 140, true, true, 0.0,
                    12.0, 4.0, false, true, 0, 0, 0,
                    "Same spot, exactly 8 fuel.",
                    "", "THE measured threshold. The code says 16; the engine flips here."),
            row("B3", "AUTONOMOUS_CYCLER", 0.40, 4.03, 15, 140, true, true, 0.0,
                    12.0, 4.0, false, true, 0, 0, 0,
                    "Same spot, 15 fuel.",
                    "", "Between the measured 8 and the coded 16. Expect no change from B2."),
            row("B4", "AUTONOMOUS_CYCLER", 0.40, 4.03, 16, 140, true, true, 0.0,
                    12.0, 4.0, false, true, 0, 0, 0,
                    "Same spot, 16 fuel — the value minFuelToScore actually specifies.",
                    "", "If B2 and B4 behave identically the 16 rung is dead code."),
            row("B5", "AUTONOMOUS_CYCLER", 9.50, 4.03, 8, 140, true, true, 0.0,
                    12.0, 4.0, false, true, 0, 0, 0,
                    "4.87 m from hub — past the centerline into the opponent half, 8 fuel.",
                    "", "Same out-of-range condition as B2 but a long trip home. Distance is\n"
                        + "not in the utility at all; does the bot care?"),
            row("B6", "AUTONOMOUS_CYCLER", 2.0, 4.03, 1, 140, true, true, 0.0,
                    12.0, 4.0, false, true, 0, 0, 0,
                    "In range (2.63 m), hub live, 1 fuel.",
                    "", "minFuelToScore is 1 in range, so this should commit on one ball."),

            // --- C: the last window (the live finding) ------------------------
            // SHIFT1 = ours live and about to go dark. SHIFT2 = ours dark and
            // about to open. C3 vs C6 is the matched pair that isolates it.
            row("C1", "AUTONOMOUS_CYCLER", 2.0, 4.03, 0, 108, true, false, 3.0,
                    12.0, 4.0, false, true, 0, 0, 0,
                    "SHIFT1, LAST WINDOW, empty, 3 s to the flip.",
                    "", "Baseline of the closing quadrant with nothing in hand."),
            row("C2", "AUTONOMOUS_CYCLER", 2.0, 4.03, 8, 108, true, false, 3.0,
                    12.0, 4.0, false, true, 0, 0, 0,
                    "SHIFT1, LAST WINDOW, 8 fuel, 3 s to the flip.",
                    "", "Exactly the harvest-force threshold, with the hub about to die."),
            row("C3", "AUTONOMOUS_CYCLER", 2.0, 4.03, 20, 108, true, false, 3.0,
                    12.0, 4.0, false, true, 0, 0, 0,
                    "SHIFT1, LAST WINDOW, 20 fuel, 3 s to the flip.",
                    "", "THE open question. Staging at the arc is the intuitive play with a\n"
                        + "full hopper 3 s from losing the hub; the engine currently cycles.\n"
                        + "willOwnHubDeactivate() exists but nothing reads it."),
            row("C4", "ADAPTIVE_COMPETITOR", 2.0, 4.03, 20, 108, true, false, 3.0,
                    12.0, 4.0, false, true, 0, 0, 0,
                    "Same as C3 but ADAPTIVE_COMPETITOR (no archetype modifier).",
                    "", "Isolates archetype handling from hub state."),
            row("C5", "TACTICAL_DEFENDER", 2.0, 4.03, 20, 108, true, false, 3.0,
                    12.0, 4.0, false, true, 0, 0, 0,
                    "Same as C3 but TACTICAL_DEFENDER.",
                    "", "Defender in its own scoring window, 3 s from losing it. Does the\n"
                        + "0.99 defensive utility still beat the harvest force's 0.98?"),
            row("C6", "AUTONOMOUS_CYCLER", 2.0, 4.03, 20, 83, false, true, 3.0,
                    12.0, 4.0, false, true, 0, 0, 0,
                    "SHIFT2, OUR hub dark and about to OPEN, 20 fuel, 3 s to the flip.",
                    "", "CONTROL for C3: same hopper, same countdown, opposite hub state.\n"
                        + "If this stages and C3 does not, the last-window gap is proven."),

            // --- D: staging while the hub is dark ------------------------------
            row("D1", "ADAPTIVE_COMPETITOR", 2.0, 4.03, 20, 90, false, true, 10.0,
                    12.0, 4.0, false, true, 0, 0, 0,
                    "SHIFT2, hub dark, 10 s until it opens, 20 fuel.",
                    "", "STAGE_STANDOFF is 0.80 but VACUUM is 0.88-0.98 and STOCKPILE\n"
                        + "0.86-0.96, so harvesting wins. Observed: vacuums to 30 before staging."),
            row("D2", "ADAPTIVE_COMPETITOR", 2.0, 4.03, 20, 83, false, true, 3.0,
                    12.0, 4.0, false, true, 0, 0, 0,
                    "Same, but 3 s until the hub opens.",
                    "", "STAGE should jump 0.80 -> 0.95 to anticipate. Does it beat VACUUM?"),
            row("D3", "ADAPTIVE_COMPETITOR", 2.0, 4.03, 30, 90, false, true, 10.0,
                    12.0, 4.0, false, true, 0, 0, 0,
                    "SHIFT2, hub dark, 10 s to open, hopper FULL.",
                    "", "Boundary: 30 is where D1 finally starts staging."),
            row("D4", "ADAPTIVE_COMPETITOR", 2.0, 4.03, 18, 90, false, true, 10.0,
                    12.0, 4.0, false, true, 0, 0, 0,
                    "Same, holding exactly minFuelToStage (18).",
                    "", "Should make STAGE eligible even though VACUUM still outranks it."),
            row("D5", "ADAPTIVE_COMPETITOR", 2.0, 4.03, 17, 90, false, true, 10.0,
                    12.0, 4.0, false, true, 0, 0, 0,
                    "Same, one below minFuelToStage (17).",
                    "", "Control for D4 — STAGE should be ineligible here."),

            // --- E: endgame climb ---------------------------------------------
            // RUSH_CLIMB is piecewise in matchTime: 0.99+0.01*(1-t/15) for t<=15,
            // and 0.85+0.13*(1-(t-15)/5) for 15<t<=20. That is a DISCONTINUITY at
            // t=15: the value jumps from 0.954 at t=16 to 0.99 at t=15. Since
            // CYCLE_SCORE_HUB sits at 0.98, the bot declines to climb in the 16-20 s
            // band and starts at 15 s. E2/E3 isolate that step.
            row("E1", "CO_PILOT", 2.0, 4.03, 12, 20, true, true, 0.0,
                    12.0, 4.0, false, true, 0, 0, 0,
                    "ENDGAME, 20 s left, CO_PILOT holding 12 fuel.",
                    "", "Lowest point of the climb ramp (0.85). Cycle should win."),
            row("E2", "CO_PILOT", 2.0, 4.03, 12, 16, true, true, 0.0,
                    12.0, 4.0, false, true, 0, 0, 0,
                    "ENDGAME, 16 s left, same state.",
                    "", "Ramp gives 0.954, still under CYCLE's 0.98. One second later it flips."),
            row("E3", "CO_PILOT", 2.0, 4.03, 12, 15, true, true, 0.0,
                    12.0, 4.0, false, true, 0, 0, 0,
                    "ENDGAME, 15 s left, same state.",
                    "", "Ramp jumps to 0.99 and overtakes CYCLE. E2 vs E3 is the step."),
            row("E4", "CO_PILOT", 2.0, 4.03, 12, 10, true, true, 0.0,
                    12.0, 4.0, false, true, 0, 0, 0,
                    "ENDGAME, 10 s left, same state.",
                    "", "Should be climbing by now. Abandoning 12 points of fuel — right call?"),
            row("E5", "CO_PILOT", 2.0, 4.03, 0, 15, true, true, 0.0,
                    12.0, 4.0, false, true, 0, 0, 0,
                    "ENDGAME, 15 s left, empty-handed.",
                    "", "Control for E3: climbing with nothing in the hopper is free."),
            row("E6", "AUTONOMOUS_CYCLER", 2.0, 4.03, 12, 15, true, true, 0.0,
                    12.0, 4.0, false, true, 0, 0, 0,
                    "ENDGAME, 15 s left, AUTONOMOUS_CYCLER with 12 fuel.",
                    "", "Control for E3: the climb gate is CO_PILOT-only, so this must NOT climb."),

            // --- F: defenders (the margin is only ~0.01) -----------------------
            row("F1", "TACTICAL_DEFENDER", 2.5, 4.03, 20, 120, true, false, 15.0,
                    12.0, 4.0, false, true, 0, 0, 0,
                    "SHIFT1, our hub live, defender holding 20 fuel, opponent hub dark.",
                    "", "DENY/LEAD beat the harvest force by ~0.01. Pin the behaviour."),
            row("F2", "DEFENSE_BULLY", 2.5, 4.03, 20, 120, true, false, 15.0,
                    12.0, 4.0, false, true, 0, 0, 0,
                    "Same but DEFENSE_BULLY.",
                    "", "Second defender, same margin question. LEAD_INTERCEPT is 0.99."),
            row("F3", "TACTICAL_DEFENDER", 2.5, 4.03, 20, 100, false, true, 20.0,
                    12.0, 4.0, false, true, 0, 0, 0,
                    "SHIFT2, our hub dark, defender holding 20 fuel, opponent hub live.",
                    "", "Opposite quadrant. With our hub dark the harvest force cannot fire\n"
                        + "for them, so this should be a comfortable defensive pick."),
            row("F4", "TACTICAL_DEFENDER", 2.5, 4.03, 0, 120, true, false, 15.0,
                    12.0, 4.0, false, true, 0, 0, 0,
                    "SHIFT1, defender EMPTY.",
                    "", "Control: with no fuel the harvest force is inert, so this isolates\n"
                        + "the archetype modifier from the :551 interaction."),

            // --- G: geometry anchors for inShootingRange -----------------------
            // Same fuel, same clock, walking the bot across the range boundary.
            row("G1", "AUTONOMOUS_CYCLER", 0.40, 4.03, 8, 140, true, true, 0.0,
                    12.0, 4.0, false, true, 0, 0, 0,
                    "x=0.40, 4.23 m from the hub — OUTSIDE the 4.0 m radius.",
                    "", "Geometry anchor for the far edge of the home zone."),
            row("G2", "AUTONOMOUS_CYCLER", 4.00, 4.03, 8, 140, true, true, 0.0,
                    12.0, 4.0, false, true, 0, 0, 0,
                    "x=4.00, 0.63 m from the hub — deep in range, on the standoff arc.",
                    "", "Geometry anchor for the near edge."),
            row("G3", "AUTONOMOUS_CYCLER", 8.00, 4.03, 8, 140, true, true, 0.0,
                    12.0, 4.0, false, true, 0, 0, 0,
                    "x=8.00, 3.37 m from the hub — STILL in range, near the centerline.",
                    "", "This is the finding: 3.37 m < 4.0 m, so minFuelToScore is 1\n"
                        + "essentially everywhere a bot can legally be in its own half."),
            row("G4", "AUTONOMOUS_CYCLER", 9.50, 4.03, 8, 140, true, true, 0.0,
                    12.0, 4.0, false, true, 0, 0, 0,
                    "x=9.50, 4.87 m from the hub — outside the radius, in the opponent half.",
                    "", "Only here does the 16 rung of minFuelToScore become reachable."),
        };
        for (Object[] r : rows) sb.append(String.join("\t", str(r))).append('\n');
        Files.writeString(tsv, sb.toString(), StandardCharsets.UTF_8);
    }

    private static Object[] row(String id, String arch, double x, double y, int held, double t,
            boolean hub, boolean oppHub, double shift, double ox, double oy, boolean auto,
            boolean obs, int diff, int oppPieces, int alliesHeld, String situation,
            String expected, String notes) {
        // Template rows use honest defaults: empty field, default hardware,
        // legacy roster derivation. Hand-authored rows can append eight tail
        // columns to override.
        return new Object[] {id, arch, x, y, held, t, hub, oppHub, shift, ox, oy, auto, obs,
                diff, oppPieces, alliesHeld, situation, expected, notes,
                0, 0, 0, true, 30, false, "", ""};
    }

    private static String[] str(Object[] row) {
        String[] out = new String[row.length];
        for (int i = 0; i < row.length; i++) {
            Object v = row[i];
            String s = String.valueOf(v);
            out[i] = s.replace("\t", " ").replace("\n", " ");
        }
        return out;
    }
}
