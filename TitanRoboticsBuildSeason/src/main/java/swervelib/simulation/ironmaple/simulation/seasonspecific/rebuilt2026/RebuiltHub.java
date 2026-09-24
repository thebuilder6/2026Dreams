package swervelib.simulation.ironmaple.simulation.seasonspecific.rebuilt2026;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.MetersPerSecond;

import edu.wpi.first.math.geometry.*;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import swervelib.simulation.ironmaple.simulation.Goal;
import swervelib.simulation.ironmaple.simulation.gamepieces.GamePiece;

import java.util.*;

/**
 * Simulates a HUB on the 2026 Rebuilt field where FUEL can be scored.
 * Fixed chute spawn positions and directions:
 * - Blue Hub shoots towards the Blue alliance side (-X, back to alliance territory).
 * - Red Hub shoots towards the Red alliance side (+X, back to alliance territory).
 * - Corrected 3D mirroring so Red poses do not rotate into negative coordinates.
 */
public class RebuiltHub extends Goal {

    public static final Translation3d blueHubPose = new Translation3d(4.5974, 4.034536, 1.5748);
    public static final Translation3d redHubPose = new Translation3d(11.938, 4.034536, 1.5748);

    // Blue chutes: on the east side (+X) of the Blue hub, facing east (yaw ~0 deg) rolling into the center
    protected static final Pose3d[] blueShootPoses = {
        new Pose3d(
                blueHubPose.plus(new Translation3d(0.5969, 0.447675, -0.5)),
                new Rotation3d(Degrees.of(0), Degrees.of(-5), Degrees.of(33.75))),
        new Pose3d(
                blueHubPose.plus(new Translation3d(0.5969, 0.149225, -0.5)),
                new Rotation3d(Degrees.of(0), Degrees.of(-5), Degrees.of(11.25))),
        new Pose3d(
                blueHubPose.plus(new Translation3d(0.5969, -0.149225, -0.5)),
                new Rotation3d(Degrees.of(0), Degrees.of(-5), Degrees.of(-11.25))),
        new Pose3d(
                blueHubPose.plus(new Translation3d(0.5969, -0.447675, -0.5)),
                new Rotation3d(Degrees.of(0), Degrees.of(-5), Degrees.of(-33.75)))
    };

    // Red chutes: on the west side (-X) of the Red hub, facing west (yaw ~180 deg) rolling into the center
    protected static final Pose3d[] redShootPoses = {
        new Pose3d(
                redHubPose.plus(new Translation3d(-0.5969, -0.447675, -0.5)),
                new Rotation3d(Degrees.of(0), Degrees.of(-5), Degrees.of(180.0 - 33.75))),
        new Pose3d(
                redHubPose.plus(new Translation3d(-0.5969, -0.149225, -0.5)),
                new Rotation3d(Degrees.of(0), Degrees.of(-5), Degrees.of(180.0 - 11.25))),
        new Pose3d(
                redHubPose.plus(new Translation3d(-0.5969, 0.149225, -0.5)),
                new Rotation3d(Degrees.of(0), Degrees.of(-5), Degrees.of(180.0 + 11.25))),
        new Pose3d(
                redHubPose.plus(new Translation3d(-0.5969, 0.447675, -0.5)),
                new Rotation3d(Degrees.of(0), Degrees.of(-5), Degrees.of(180.0 + 33.75)))
    };

    public static final double GoalRadius = 0.5969;
    static final Random rng = new Random();

    StructPublisher<Pose3d> posePublisher;
    protected final Arena2026Rebuilt arena;

    public RebuiltHub(Arena2026Rebuilt arena, boolean isBlue) {
        super(
                arena,
                Inches.of(47),
                Inches.of(47),
                Inches.of(10),
                "Fuel",
                isBlue ? blueHubPose : redHubPose,
                isBlue,
                false);

        this.arena = arena;
        StructPublisher<Pose3d> hubPosePublisher = NetworkTableInstance.getDefault()
                .getStructTopic(isBlue ? "BlueHub" : "RedHub", Pose3d.struct)
                .publish();
        hubPosePublisher.set(new Pose3d(position, new Rotation3d()));
    }

    @Override
    protected boolean checkVel(GamePiece gamePiece) {
        return gamePiece.getVelocity3dMPS().getZ() < 0;
    }

    @Override
    protected boolean checkCollision(GamePiece gamePiece) {
        return Math.pow(gamePiece.getPose3d().getX() - position.getX(), 2)
                + Math.pow(gamePiece.getPose3d().getY() - position.getY(), 2)
                + Math.pow(gamePiece.getPose3d().getZ() - position.getZ(), 2)
                < Math.pow(GoalRadius, 2);
    }

    @Override
    protected void addPoints() {
        arena.addValueToMatchBreakdown(isBlue, "TotalFuelInHub", 1);
        arena.addValueToMatchBreakdown(isBlue, "WastedFuel", arena.isActive(isBlue) ? 0 : 1);
        arena.addToScore(isBlue, arena.isActive(isBlue) ? 1 : 0);

        Pose3d shootPose = isBlue ? blueShootPoses[rng.nextInt(4)] : redShootPoses[rng.nextInt(4)];

        arena.addPieceWithVariance(
                shootPose.getTranslation().toTranslation2d(),
                new Rotation2d(shootPose.getRotation().getZ()),
                shootPose.getMeasureZ(),
                MetersPerSecond.of(2.2),
                shootPose.getRotation().getMeasureY(),
                0.02,
                0.02,
                15.0,
                0.2,
                5.0);
    }

    @Override
    public void draw(List<Pose3d> drawList) {
    }
}
