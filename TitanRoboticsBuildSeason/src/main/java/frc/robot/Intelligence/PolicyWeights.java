package frc.robot.Intelligence;

import java.util.HashMap;
import java.util.Map;

/**
 * Immutable configuration record parameterizing all utility weights and strategic
 * thresholds evaluated in {@link JevDecisionEngine}.
 *
 * <p>Enables the automated headless match scoring rig ({@code tools/score/}) and
 * parameter sweeps to optimize tactical weights without requiring code changes.
 * Supports parsing from comma-delimited {@code key=value} strings (e.g., via
 * {@code -Dfrc.jev.weights=...} or CLI arguments), enforcing strict validation where
 * unknown keys throw {@link IllegalArgumentException}.
 */
public record PolicyWeights(
        // Climb Utility (Endgame)
        double climbBase15,
        double climbScale15,
        double climbBase20,
        double climbScale20,

        // Cycle Score Hub
        double scoreHubBase,
        double scoreHubScale,
        double scoreHubBehindBonus,
        double scoreHubBehindMax,
        double scoreHubCapacityDivisor,
        int scoreHubMinFuelNormal,
        int scoreHubMinFuelShiftEnding,
        double scoreHubShootingRangeMeters,
        double scoreHubShiftEndingWindowSec,

        // Stage Standoff
        double stageStandoffBase,
        double stageShiftImminentCoPilot,
        double stageShiftImminentNormal,
        double stageShiftWindowSec,
        int stageMinFuelNormal,

        // Vacuum Midfield
        double vacuumInactiveBase,
        double vacuumInactiveScale,
        double vacuumActiveBase,
        double vacuumActiveScale,
        int vacuumActiveNormalBatch,
        int vacuumActiveShiftEndingBatch,
        double vacuumActiveFullCap,

        // Stockpile Depot
        double stockpileDepotBase,
        double stockpileDepotScale,

        // Sweep Alliance Zone
        double sweepAllianceZoneActive,
        double sweepAllianceZoneInactiveBase,
        double sweepAllianceZoneInactiveScale,

        // Poach Opponent Zone
        double poachOpponentZoneUtility,
        double poachOpponentZoneWindowSec,
        int poachOpponentZoneMaxHeld,

        // Shuttle Pass
        double shuttlePassUtility,
        double shuttlePassMinDistMeters,
        int shuttlePassMinHeld,

        // Long Range Snipe
        double snipeCloseUtility,
        double snipeFarUtility,
        double snipeMinDistMeters,
        int snipeMinHeld,

        // Defense Objectives
        double laneDenialActiveUtility,
        double laneDenialMaxDistMeters,
        double shadowMidlineBaseUtility,
        double interceptBaseUtility,
        double chokeTrenchUtility,
        double screenForAllyUtility,

        // Archetype Adjustments & Overrides
        double tacticalDefenderLaneMultiplier,
        double tacticalDefenderShadowMultiplier,
        double bullyInterceptUtility,
        double coPilotActiveScoreUtility,
        double harvestDeadlineForceUtility,
        double autoBatchDumpScoreUtility,
        double autoHarvestVacuumUtility,

        // Action Inertia & Commitment (Dave Mark)
        double commitmentMargin,
        double commitmentDecisiveMargin,
        double commitmentMinHoldSec,
        double inertiaInitialBoost,
        double inertiaTimeConstantSec,
        double inertiaResidualMargin,

        // Spatial Fuel Clustering & Scent (EQS)
        double clusterNeighborhoodRadius,
        double clusterKernelSigma,
        double clusterDensityExponent,
        double clusterDistanceFloor,
        double harvestHeadingAlignScale,
        double harvestReturnVectorBonus
) {

    /**
     * Backward-compatible 53-parameter constructor defaulting action inertia and scent parameters.
     */
    public PolicyWeights(
            double climbBase15, double climbScale15, double climbBase20, double climbScale20,
            double scoreHubBase, double scoreHubScale, double scoreHubBehindBonus, double scoreHubBehindMax,
            double scoreHubCapacityDivisor, int scoreHubMinFuelNormal, int scoreHubMinFuelShiftEnding,
            double scoreHubShootingRangeMeters, double scoreHubShiftEndingWindowSec,
            double stageStandoffBase, double stageShiftImminentCoPilot, double stageShiftImminentNormal,
            double stageShiftWindowSec, int stageMinFuelNormal,
            double vacuumInactiveBase, double vacuumInactiveScale, double vacuumActiveBase, double vacuumActiveScale,
            int vacuumActiveNormalBatch, int vacuumActiveShiftEndingBatch, double vacuumActiveFullCap,
            double stockpileDepotBase, double stockpileDepotScale,
            double sweepAllianceZoneActive, double sweepAllianceZoneInactiveBase, double sweepAllianceZoneInactiveScale,
            double poachOpponentZoneUtility, double poachOpponentZoneWindowSec, int poachOpponentZoneMaxHeld,
            double shuttlePassUtility, double shuttlePassMinDistMeters, int shuttlePassMinHeld,
            double snipeCloseUtility, double snipeFarUtility, double snipeMinDistMeters, int snipeMinHeld,
            double laneDenialActiveUtility, double laneDenialMaxDistMeters, double shadowMidlineBaseUtility,
            double interceptBaseUtility, double chokeTrenchUtility, double screenForAllyUtility,
            double tacticalDefenderLaneMultiplier, double tacticalDefenderShadowMultiplier, double bullyInterceptUtility,
            double coPilotActiveScoreUtility, double harvestDeadlineForceUtility, double autoBatchDumpScoreUtility,
            double autoHarvestVacuumUtility) {
        this(
                climbBase15, climbScale15, climbBase20, climbScale20,
                scoreHubBase, scoreHubScale, scoreHubBehindBonus, scoreHubBehindMax, scoreHubCapacityDivisor,
                scoreHubMinFuelNormal, scoreHubMinFuelShiftEnding, scoreHubShootingRangeMeters, scoreHubShiftEndingWindowSec,
                stageStandoffBase, stageShiftImminentCoPilot, stageShiftImminentNormal, stageShiftWindowSec, stageMinFuelNormal,
                vacuumInactiveBase, vacuumInactiveScale, vacuumActiveBase, vacuumActiveScale,
                vacuumActiveNormalBatch, vacuumActiveShiftEndingBatch, vacuumActiveFullCap,
                stockpileDepotBase, stockpileDepotScale,
                sweepAllianceZoneActive, sweepAllianceZoneInactiveBase, sweepAllianceZoneInactiveScale,
                poachOpponentZoneUtility, poachOpponentZoneWindowSec, poachOpponentZoneMaxHeld,
                shuttlePassUtility, shuttlePassMinDistMeters, shuttlePassMinHeld,
                snipeCloseUtility, snipeFarUtility, snipeMinDistMeters, snipeMinHeld,
                laneDenialActiveUtility, laneDenialMaxDistMeters, shadowMidlineBaseUtility,
                interceptBaseUtility, chokeTrenchUtility, screenForAllyUtility,
                tacticalDefenderLaneMultiplier, tacticalDefenderShadowMultiplier, bullyInterceptUtility,
                coPilotActiveScoreUtility, harvestDeadlineForceUtility, autoBatchDumpScoreUtility, autoHarvestVacuumUtility,
                0.06, 0.20, 1.5, 0.20, 1.0, 0.04,
                1.30, 0.50, 1.50, 0.40, 0.30, 0.35
        );
    }

    /**
     * Bit-identical default weights matching existing production Jev AI policy.
     */
    public static final PolicyWeights DEFAULT = new PolicyWeights(
            // Climb
            0.99, 0.01, 0.85, 0.13,
            // Score Hub
            0.72, 0.26, 0.03, 0.99, 20.0, 16, 4, 4.0, 4.5,
            // Stage Standoff
            0.80, 0.99, 0.95, 3.5, 18,
            // Vacuum Midfield
            0.88, 0.10, 0.82, 0.14, 18, 6, 0.35,
            // Stockpile Depot
            0.86, 0.10,
            // Sweep Alliance Zone
            0.96, 0.90, 0.08,
            // Poach Opponent Zone
            0.78, 6.0, 20,
            // Shuttle Pass
            0.87, 6.0, 16,
            // Long Range Snipe
            0.94, 0.86, 3.6, 6,
            // Defense
            0.86, 6.5, 0.65, 0.75, 0.91, 0.89,
            // Archetype Overrides
            1.15, 1.10, 0.99, 0.98, 0.98, 0.95, 0.85,
            // Action Inertia
            0.06, 0.20, 1.5, 0.20, 1.0, 0.04,
            // Scent & Clustering
            1.30, 0.50, 1.50, 0.40, 0.30, 0.35
    );


    private static volatile PolicyWeights activeWeights = loadFromSystemProperties();

    /**
     * Returns the currently active PolicyWeights (either loaded from system properties or default).
     */
    public static PolicyWeights getActive() {
        return activeWeights;
    }

    /**
     * Programmatically sets the active policy weights for testing or sweeps.
     */
    public static void setActive(PolicyWeights weights) {
        activeWeights = (weights != null) ? weights : DEFAULT;
    }

    /**
     * Resets the active policy weights to {@link #DEFAULT}.
     */
    public static void resetToDefault() {
        activeWeights = DEFAULT;
    }

    private static PolicyWeights loadFromSystemProperties() {
        String prop = System.getProperty("frc.jev.weights");
        if (prop == null || prop.trim().isEmpty()) {
            prop = System.getenv("FRC_JEV_WEIGHTS");
        }
        if (prop != null && !prop.trim().isEmpty()) {
            return fromString(prop);
        }
        return DEFAULT;
    }

    /**
     * Parses key-value pairs (e.g. {@code "scoreHubBase=0.75,vacuumActiveBase=0.80"}) on top of {@link #DEFAULT}.
     * Unknown keys throw {@link IllegalArgumentException}.
     */
    public static PolicyWeights fromString(String spec) {
        return DEFAULT.applyOverrides(spec);
    }

    /**
     * Returns a new {@link PolicyWeights} by applying comma-separated key=value overrides.
     *
     * @param spec comma-separated list of {@code key=value} or semicolon-separated pairs
     * @throws IllegalArgumentException if an unknown key is encountered or value cannot be parsed
     */
    public PolicyWeights applyOverrides(String spec) {
        if (spec == null || spec.trim().isEmpty()) {
            return this;
        }

        Builder builder = toBuilder();
        String[] pairs = spec.split("[,;]");
        for (String pair : pairs) {
            String trimmed = pair.trim();
            if (trimmed.isEmpty()) {
                continue;
            }
            int eq = trimmed.indexOf('=');
            if (eq <= 0 || eq >= trimmed.length() - 1) {
                throw new IllegalArgumentException("Malformed key=value weight spec: '" + trimmed + "'");
            }
            String key = trimmed.substring(0, eq).trim();
            String valStr = trimmed.substring(eq + 1).trim();

            builder.setField(key, valStr);
        }
        return builder.build();
    }

    /**
     * Converts this instance into a mutable {@link Builder}.
     */
    public Builder toBuilder() {
        return new Builder(this);
    }

    /**
     * Builder for constructing custom {@link PolicyWeights}.
     */
    public static final class Builder {
        private double climbBase15;
        private double climbScale15;
        private double climbBase20;
        private double climbScale20;
        private double scoreHubBase;
        private double scoreHubScale;
        private double scoreHubBehindBonus;
        private double scoreHubBehindMax;
        private double scoreHubCapacityDivisor;
        private int scoreHubMinFuelNormal;
        private int scoreHubMinFuelShiftEnding;
        private double scoreHubShootingRangeMeters;
        private double scoreHubShiftEndingWindowSec;
        private double stageStandoffBase;
        private double stageShiftImminentCoPilot;
        private double stageShiftImminentNormal;
        private double stageShiftWindowSec;
        private int stageMinFuelNormal;
        private double vacuumInactiveBase;
        private double vacuumInactiveScale;
        private double vacuumActiveBase;
        private double vacuumActiveScale;
        private int vacuumActiveNormalBatch;
        private int vacuumActiveShiftEndingBatch;
        private double vacuumActiveFullCap;
        private double stockpileDepotBase;
        private double stockpileDepotScale;
        private double sweepAllianceZoneActive;
        private double sweepAllianceZoneInactiveBase;
        private double sweepAllianceZoneInactiveScale;
        private double poachOpponentZoneUtility;
        private double poachOpponentZoneWindowSec;
        private int poachOpponentZoneMaxHeld;
        private double shuttlePassUtility;
        private double shuttlePassMinDistMeters;
        private int shuttlePassMinHeld;
        private double snipeCloseUtility;
        private double snipeFarUtility;
        private double snipeMinDistMeters;
        private int snipeMinHeld;
        private double laneDenialActiveUtility;
        private double laneDenialMaxDistMeters;
        private double shadowMidlineBaseUtility;
        private double interceptBaseUtility;
        private double chokeTrenchUtility;
        private double screenForAllyUtility;
        private double tacticalDefenderLaneMultiplier;
        private double tacticalDefenderShadowMultiplier;
        private double bullyInterceptUtility;
        private double coPilotActiveScoreUtility;
        private double harvestDeadlineForceUtility;
        private double autoBatchDumpScoreUtility;
        private double autoHarvestVacuumUtility;
        private double commitmentMargin;
        private double commitmentDecisiveMargin;
        private double commitmentMinHoldSec;
        private double inertiaInitialBoost;
        private double inertiaTimeConstantSec;
        private double inertiaResidualMargin;
        private double clusterNeighborhoodRadius;
        private double clusterKernelSigma;
        private double clusterDensityExponent;
        private double clusterDistanceFloor;
        private double harvestHeadingAlignScale;
        private double harvestReturnVectorBonus;

        public Builder(PolicyWeights base) {
            this.climbBase15 = base.climbBase15;
            this.climbScale15 = base.climbScale15;
            this.climbBase20 = base.climbBase20;
            this.climbScale20 = base.climbScale20;
            this.scoreHubBase = base.scoreHubBase;
            this.scoreHubScale = base.scoreHubScale;
            this.scoreHubBehindBonus = base.scoreHubBehindBonus;
            this.scoreHubBehindMax = base.scoreHubBehindMax;
            this.scoreHubCapacityDivisor = base.scoreHubCapacityDivisor;
            this.scoreHubMinFuelNormal = base.scoreHubMinFuelNormal;
            this.scoreHubMinFuelShiftEnding = base.scoreHubMinFuelShiftEnding;
            this.scoreHubShootingRangeMeters = base.scoreHubShootingRangeMeters;
            this.scoreHubShiftEndingWindowSec = base.scoreHubShiftEndingWindowSec;
            this.stageStandoffBase = base.stageStandoffBase;
            this.stageShiftImminentCoPilot = base.stageShiftImminentCoPilot;
            this.stageShiftImminentNormal = base.stageShiftImminentNormal;
            this.stageShiftWindowSec = base.stageShiftWindowSec;
            this.stageMinFuelNormal = base.stageMinFuelNormal;
            this.vacuumInactiveBase = base.vacuumInactiveBase;
            this.vacuumInactiveScale = base.vacuumInactiveScale;
            this.vacuumActiveBase = base.vacuumActiveBase;
            this.vacuumActiveScale = base.vacuumActiveScale;
            this.vacuumActiveNormalBatch = base.vacuumActiveNormalBatch;
            this.vacuumActiveShiftEndingBatch = base.vacuumActiveShiftEndingBatch;
            this.vacuumActiveFullCap = base.vacuumActiveFullCap;
            this.stockpileDepotBase = base.stockpileDepotBase;
            this.stockpileDepotScale = base.stockpileDepotScale;
            this.sweepAllianceZoneActive = base.sweepAllianceZoneActive;
            this.sweepAllianceZoneInactiveBase = base.sweepAllianceZoneInactiveBase;
            this.sweepAllianceZoneInactiveScale = base.sweepAllianceZoneInactiveScale;
            this.poachOpponentZoneUtility = base.poachOpponentZoneUtility;
            this.poachOpponentZoneWindowSec = base.poachOpponentZoneWindowSec;
            this.poachOpponentZoneMaxHeld = base.poachOpponentZoneMaxHeld;
            this.shuttlePassUtility = base.shuttlePassUtility;
            this.shuttlePassMinDistMeters = base.shuttlePassMinDistMeters;
            this.shuttlePassMinHeld = base.shuttlePassMinHeld;
            this.snipeCloseUtility = base.snipeCloseUtility;
            this.snipeFarUtility = base.snipeFarUtility;
            this.snipeMinDistMeters = base.snipeMinDistMeters;
            this.snipeMinHeld = base.snipeMinHeld;
            this.laneDenialActiveUtility = base.laneDenialActiveUtility;
            this.laneDenialMaxDistMeters = base.laneDenialMaxDistMeters;
            this.shadowMidlineBaseUtility = base.shadowMidlineBaseUtility;
            this.interceptBaseUtility = base.interceptBaseUtility;
            this.chokeTrenchUtility = base.chokeTrenchUtility;
            this.screenForAllyUtility = base.screenForAllyUtility;
            this.tacticalDefenderLaneMultiplier = base.tacticalDefenderLaneMultiplier;
            this.tacticalDefenderShadowMultiplier = base.tacticalDefenderShadowMultiplier;
            this.bullyInterceptUtility = base.bullyInterceptUtility;
            this.coPilotActiveScoreUtility = base.coPilotActiveScoreUtility;
            this.harvestDeadlineForceUtility = base.harvestDeadlineForceUtility;
            this.autoBatchDumpScoreUtility = base.autoBatchDumpScoreUtility;
            this.autoHarvestVacuumUtility = base.autoHarvestVacuumUtility;
            this.commitmentMargin = base.commitmentMargin;
            this.commitmentDecisiveMargin = base.commitmentDecisiveMargin;
            this.commitmentMinHoldSec = base.commitmentMinHoldSec;
            this.inertiaInitialBoost = base.inertiaInitialBoost;
            this.inertiaTimeConstantSec = base.inertiaTimeConstantSec;
            this.inertiaResidualMargin = base.inertiaResidualMargin;
            this.clusterNeighborhoodRadius = base.clusterNeighborhoodRadius;
            this.clusterKernelSigma = base.clusterKernelSigma;
            this.clusterDensityExponent = base.clusterDensityExponent;
            this.clusterDistanceFloor = base.clusterDistanceFloor;
            this.harvestHeadingAlignScale = base.harvestHeadingAlignScale;
            this.harvestReturnVectorBonus = base.harvestReturnVectorBonus;
        }

        public void setField(String key, String valStr) {
            try {
                switch (key) {
                    case "climbBase15" -> this.climbBase15 = Double.parseDouble(valStr);
                    case "climbScale15" -> this.climbScale15 = Double.parseDouble(valStr);
                    case "climbBase20" -> this.climbBase20 = Double.parseDouble(valStr);
                    case "climbScale20" -> this.climbScale20 = Double.parseDouble(valStr);
                    case "scoreHubBase" -> this.scoreHubBase = Double.parseDouble(valStr);
                    case "scoreHubScale" -> this.scoreHubScale = Double.parseDouble(valStr);
                    case "scoreHubBehindBonus" -> this.scoreHubBehindBonus = Double.parseDouble(valStr);
                    case "scoreHubBehindMax" -> this.scoreHubBehindMax = Double.parseDouble(valStr);
                    case "scoreHubCapacityDivisor" -> this.scoreHubCapacityDivisor = Double.parseDouble(valStr);
                    case "scoreHubMinFuelNormal" -> this.scoreHubMinFuelNormal = Integer.parseInt(valStr);
                    case "scoreHubMinFuelShiftEnding" -> this.scoreHubMinFuelShiftEnding = Integer.parseInt(valStr);
                    case "scoreHubShootingRangeMeters" -> this.scoreHubShootingRangeMeters = Double.parseDouble(valStr);
                    case "scoreHubShiftEndingWindowSec" -> this.scoreHubShiftEndingWindowSec = Double.parseDouble(valStr);
                    case "stageStandoffBase" -> this.stageStandoffBase = Double.parseDouble(valStr);
                    case "stageShiftImminentCoPilot" -> this.stageShiftImminentCoPilot = Double.parseDouble(valStr);
                    case "stageShiftImminentNormal" -> this.stageShiftImminentNormal = Double.parseDouble(valStr);
                    case "stageShiftWindowSec" -> this.stageShiftWindowSec = Double.parseDouble(valStr);
                    case "stageMinFuelNormal" -> this.stageMinFuelNormal = Integer.parseInt(valStr);
                    case "vacuumInactiveBase" -> this.vacuumInactiveBase = Double.parseDouble(valStr);
                    case "vacuumInactiveScale" -> this.vacuumInactiveScale = Double.parseDouble(valStr);
                    case "vacuumActiveBase" -> this.vacuumActiveBase = Double.parseDouble(valStr);
                    case "vacuumActiveScale" -> this.vacuumActiveScale = Double.parseDouble(valStr);
                    case "vacuumActiveNormalBatch" -> this.vacuumActiveNormalBatch = Integer.parseInt(valStr);
                    case "vacuumActiveShiftEndingBatch" -> this.vacuumActiveShiftEndingBatch = Integer.parseInt(valStr);
                    case "vacuumActiveFullCap" -> this.vacuumActiveFullCap = Double.parseDouble(valStr);
                    case "stockpileDepotBase" -> this.stockpileDepotBase = Double.parseDouble(valStr);
                    case "stockpileDepotScale" -> this.stockpileDepotScale = Double.parseDouble(valStr);
                    case "sweepAllianceZoneActive" -> this.sweepAllianceZoneActive = Double.parseDouble(valStr);
                    case "sweepAllianceZoneInactiveBase" -> this.sweepAllianceZoneInactiveBase = Double.parseDouble(valStr);
                    case "sweepAllianceZoneInactiveScale" -> this.sweepAllianceZoneInactiveScale = Double.parseDouble(valStr);
                    case "poachOpponentZoneUtility" -> this.poachOpponentZoneUtility = Double.parseDouble(valStr);
                    case "poachOpponentZoneWindowSec" -> this.poachOpponentZoneWindowSec = Double.parseDouble(valStr);
                    case "poachOpponentZoneMaxHeld" -> this.poachOpponentZoneMaxHeld = Integer.parseInt(valStr);
                    case "shuttlePassUtility" -> this.shuttlePassUtility = Double.parseDouble(valStr);
                    case "shuttlePassMinDistMeters" -> this.shuttlePassMinDistMeters = Double.parseDouble(valStr);
                    case "shuttlePassMinHeld" -> this.shuttlePassMinHeld = Integer.parseInt(valStr);
                    case "snipeCloseUtility" -> this.snipeCloseUtility = Double.parseDouble(valStr);
                    case "snipeFarUtility" -> this.snipeFarUtility = Double.parseDouble(valStr);
                    case "snipeMinDistMeters" -> this.snipeMinDistMeters = Double.parseDouble(valStr);
                    case "snipeMinHeld" -> this.snipeMinHeld = Integer.parseInt(valStr);
                    case "laneDenialActiveUtility" -> this.laneDenialActiveUtility = Double.parseDouble(valStr);
                    case "laneDenialMaxDistMeters" -> this.laneDenialMaxDistMeters = Double.parseDouble(valStr);
                    case "shadowMidlineBaseUtility" -> this.shadowMidlineBaseUtility = Double.parseDouble(valStr);
                    case "interceptBaseUtility" -> this.interceptBaseUtility = Double.parseDouble(valStr);
                    case "chokeTrenchUtility" -> this.chokeTrenchUtility = Double.parseDouble(valStr);
                    case "screenForAllyUtility" -> this.screenForAllyUtility = Double.parseDouble(valStr);
                    case "tacticalDefenderLaneMultiplier" -> this.tacticalDefenderLaneMultiplier = Double.parseDouble(valStr);
                    case "tacticalDefenderShadowMultiplier" -> this.tacticalDefenderShadowMultiplier = Double.parseDouble(valStr);
                    case "bullyInterceptUtility" -> this.bullyInterceptUtility = Double.parseDouble(valStr);
                    case "coPilotActiveScoreUtility" -> this.coPilotActiveScoreUtility = Double.parseDouble(valStr);
                    case "harvestDeadlineForceUtility" -> this.harvestDeadlineForceUtility = Double.parseDouble(valStr);
                    case "autoBatchDumpScoreUtility" -> this.autoBatchDumpScoreUtility = Double.parseDouble(valStr);
                    case "autoHarvestVacuumUtility" -> this.autoHarvestVacuumUtility = Double.parseDouble(valStr);
                    case "commitmentMargin" -> this.commitmentMargin = Double.parseDouble(valStr);
                    case "commitmentDecisiveMargin" -> this.commitmentDecisiveMargin = Double.parseDouble(valStr);
                    case "commitmentMinHoldSec" -> this.commitmentMinHoldSec = Double.parseDouble(valStr);
                    case "inertiaInitialBoost" -> this.inertiaInitialBoost = Double.parseDouble(valStr);
                    case "inertiaTimeConstantSec" -> this.inertiaTimeConstantSec = Double.parseDouble(valStr);
                    case "inertiaResidualMargin" -> this.inertiaResidualMargin = Double.parseDouble(valStr);
                    case "clusterNeighborhoodRadius" -> this.clusterNeighborhoodRadius = Double.parseDouble(valStr);
                    case "clusterKernelSigma" -> this.clusterKernelSigma = Double.parseDouble(valStr);
                    case "clusterDensityExponent" -> this.clusterDensityExponent = Double.parseDouble(valStr);
                    case "clusterDistanceFloor" -> this.clusterDistanceFloor = Double.parseDouble(valStr);
                    case "harvestHeadingAlignScale" -> this.harvestHeadingAlignScale = Double.parseDouble(valStr);
                    case "harvestReturnVectorBonus" -> this.harvestReturnVectorBonus = Double.parseDouble(valStr);
                    default -> throw new IllegalArgumentException("Unknown policy weight key: '" + key + "'");
                }
            } catch (NumberFormatException e) {
                throw new IllegalArgumentException("Invalid numeric value for key '" + key + "': " + valStr, e);
            }
        }

        public Builder scoreHubBase(double val) { this.scoreHubBase = val; return this; }
        public Builder scoreHubScale(double val) { this.scoreHubScale = val; return this; }
        public Builder vacuumActiveBase(double val) { this.vacuumActiveBase = val; return this; }
        public Builder stageStandoffBase(double val) { this.stageStandoffBase = val; return this; }
        public Builder poachOpponentZoneUtility(double val) { this.poachOpponentZoneUtility = val; return this; }
        public Builder commitmentMargin(double val) { this.commitmentMargin = val; return this; }
        public Builder commitmentDecisiveMargin(double val) { this.commitmentDecisiveMargin = val; return this; }
        public Builder commitmentMinHoldSec(double val) { this.commitmentMinHoldSec = val; return this; }
        public Builder inertiaInitialBoost(double val) { this.inertiaInitialBoost = val; return this; }
        public Builder inertiaTimeConstantSec(double val) { this.inertiaTimeConstantSec = val; return this; }
        public Builder inertiaResidualMargin(double val) { this.inertiaResidualMargin = val; return this; }
        public Builder clusterNeighborhoodRadius(double val) { this.clusterNeighborhoodRadius = val; return this; }
        public Builder clusterKernelSigma(double val) { this.clusterKernelSigma = val; return this; }
        public Builder clusterDensityExponent(double val) { this.clusterDensityExponent = val; return this; }
        public Builder clusterDistanceFloor(double val) { this.clusterDistanceFloor = val; return this; }
        public Builder harvestHeadingAlignScale(double val) { this.harvestHeadingAlignScale = val; return this; }
        public Builder harvestReturnVectorBonus(double val) { this.harvestReturnVectorBonus = val; return this; }

        public PolicyWeights build() {
            return new PolicyWeights(
                    climbBase15, climbScale15, climbBase20, climbScale20,
                    scoreHubBase, scoreHubScale, scoreHubBehindBonus, scoreHubBehindMax, scoreHubCapacityDivisor,
                    scoreHubMinFuelNormal, scoreHubMinFuelShiftEnding, scoreHubShootingRangeMeters, scoreHubShiftEndingWindowSec,
                    stageStandoffBase, stageShiftImminentCoPilot, stageShiftImminentNormal, stageShiftWindowSec, stageMinFuelNormal,
                    vacuumInactiveBase, vacuumInactiveScale, vacuumActiveBase, vacuumActiveScale, vacuumActiveNormalBatch,
                    vacuumActiveShiftEndingBatch, vacuumActiveFullCap,
                    stockpileDepotBase, stockpileDepotScale,
                    sweepAllianceZoneActive, sweepAllianceZoneInactiveBase, sweepAllianceZoneInactiveScale,
                    poachOpponentZoneUtility, poachOpponentZoneWindowSec, poachOpponentZoneMaxHeld,
                    shuttlePassUtility, shuttlePassMinDistMeters, shuttlePassMinHeld,
                    snipeCloseUtility, snipeFarUtility, snipeMinDistMeters, snipeMinHeld,
                    laneDenialActiveUtility, laneDenialMaxDistMeters, shadowMidlineBaseUtility, interceptBaseUtility,
                    chokeTrenchUtility, screenForAllyUtility,
                    tacticalDefenderLaneMultiplier, tacticalDefenderShadowMultiplier, bullyInterceptUtility,
                    coPilotActiveScoreUtility, harvestDeadlineForceUtility, autoBatchDumpScoreUtility, autoHarvestVacuumUtility,
                    commitmentMargin, commitmentDecisiveMargin, commitmentMinHoldSec,
                    inertiaInitialBoost, inertiaTimeConstantSec, inertiaResidualMargin,
                    clusterNeighborhoodRadius, clusterKernelSigma, clusterDensityExponent, clusterDistanceFloor,
                    harvestHeadingAlignScale, harvestReturnVectorBonus
            );
        }

    }
}
