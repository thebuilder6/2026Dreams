package frc.robot.Auto;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.Auto.Actions.*;
import frc.robot.Data.Constants;
import frc.robot.Data.GlideConstants;
import frc.robot.Interfaces.Actions;
import frc.robot.Sim.JevDecisionEngine;
import frc.robot.Subsystems.*;

public class AutonomousTeleopAgent {

    public enum AgentState {
        EVALUATING,
        BALL_HUNTING,
        TRANSIT_TO_DEPOT,
        LOADING_AT_DEPOT,
        TRANSIT_TO_SHOOT,
        ALIGN_AND_SHOOT,
        TRANSIT_TO_CLIMB,
        CLIMBING,
        SAFETY_HALT
    }

    private AgentState currentState = AgentState.EVALUATING;
    private Actions currentAction = null;
    private final SwerveBase swerve = SwerveBase.getInstance();
    private final Intake intake = Intake.getInstance();
    private final Shooter shooter = Shooter.getInstance();
    private final Vision vision = Vision.getInstance();
    private final Dashboard dashboard = Dashboard.getInstance();

    // Inventory threshold (target capacity)
    private static final int MAX_FUEL_CAPACITY = 8;
    private int estimatedHeldBalls = 0;

    public void update() {
        double matchTime = DriverStation.getMatchTime();
        boolean isHubActive = dashboard.isHubActive();
        Pose2d currentPose = swerve.getPose();

        // ── 1. Priority 0: Endgame Transition (<= 18s remaining) ───────────
        if (matchTime > 0.0 && matchTime <= 18.0 && currentState != AgentState.CLIMBING) {
            transitionTo(AgentState.TRANSIT_TO_CLIMB);
        }

        // ── 2. Run Active Action ───────────────────────────────────────────
        if (currentAction != null) {
            currentAction.update();
            if (currentAction.isFinished()) {
                currentAction.done();
                currentAction = null;
            }
        }

        // ── 3. Executive State Machine ─────────────────────────────────────
        switch (currentState) {
            case EVALUATING:
                if (estimatedHeldBalls >= 4 && isHubActive) {
                    transitionTo(AgentState.TRANSIT_TO_SHOOT);
                } else if (vision.hasGamePiece()) {
                    transitionTo(AgentState.BALL_HUNTING);
                } else if (!isHubActive) {
                    transitionTo(AgentState.TRANSIT_TO_DEPOT);
                } else {
                    transitionTo(AgentState.BALL_HUNTING);
                }
                break;

            case BALL_HUNTING:
                // If we acquired target load or ball is no longer visible and timeout elapsed
                if (estimatedHeldBalls >= MAX_FUEL_CAPACITY) {
                    transitionTo(isHubActive ? AgentState.TRANSIT_TO_SHOOT : AgentState.TRANSIT_TO_DEPOT);
                } else if (currentAction == null || !vision.hasGamePiece()) {
                    transitionTo(AgentState.EVALUATING);
                }
                break;

            case TRANSIT_TO_DEPOT:
                if (currentAction == null) {
                    transitionTo(AgentState.LOADING_AT_DEPOT);
                }
                break;

            case LOADING_AT_DEPOT:
                // Stay at depot until full or until Hub becomes active
                intake.setState(Intake.IntakeState.INTAKING);
                if (estimatedHeldBalls >= MAX_FUEL_CAPACITY || isHubActive) {
                    transitionTo(AgentState.TRANSIT_TO_SHOOT);
                }
                break;

            case TRANSIT_TO_SHOOT:
                // Pre-spool flywheels during transit
                var solution = shooter.calculateShootingSolution(currentPose, swerve.getFieldVelocity());
                if (solution.possible()) {
                    shooter.setTargetRPM(solution.flywheelRpmLeft(), solution.flywheelRpmRight());
                    shooter.prepareToShoot();
                }

                if (currentAction == null || (solution.possible() && solution.turretAngle() != null)) {
                    transitionTo(AgentState.ALIGN_AND_SHOOT);
                }
                break;

            case ALIGN_AND_SHOOT:
                if (!isHubActive || estimatedHeldBalls == 0) {
                    shooter.stop();
                    transitionTo(AgentState.EVALUATING);
                }
                break;

            case TRANSIT_TO_CLIMB:
                if (currentAction == null) {
                    transitionTo(AgentState.CLIMBING);
                }
                break;

            case CLIMBING:
                swerve.stop();
                intake.setState(Intake.IntakeState.STANDBY);
                shooter.stop();
                break;

            case SAFETY_HALT:
            default:
                swerve.stop();
                intake.stop();
                shooter.stop();
                break;
        }
    }

    private void transitionTo(AgentState nextState) {
        if (currentAction != null) {
            currentAction.done();
            currentAction = null;
        }

        currentState = nextState;

        switch (nextState) {
            case BALL_HUNTING:
                currentAction = new BallHuntAction();
                currentAction.start();
                break;

            case TRANSIT_TO_DEPOT:
                // Auto-route to alliance depot via static pathfinder
                Pose2d depotPose = GlideConstants.GLIDE_POINTS.get(
                        dashboard.isHubActive() ? "Blue Feeder Top" : "Blue Feeder Bottom").pose();
                currentAction = new DriveToPoseAction(depotPose);
                currentAction.start();
                break;

            case TRANSIT_TO_SHOOT:
                // Move towards protected corridor behind Hub
                Pose2d shootLocation = GlideConstants.GLIDE_POINTS.get("Blue Hub Back").pose();
                currentAction = new DriveToPoseAction(shootLocation);
                currentAction.start();
                break;

            case ALIGN_AND_SHOOT:
                // Engage AutoAimAction while holding or drifting
                currentAction = new ShootAction(4.0);
                currentAction.start();
                break;

            case TRANSIT_TO_CLIMB:
                Pose2d climbSpot = GlideConstants.GLIDE_POINTS.get("Blue Right Side Climb").pose();
                currentAction = new DriveToPoseAction(climbSpot);
                currentAction.start();
                break;

            default:
                break;
        }
    }

    public void incrementBallCount() {
        estimatedHeldBalls = Math.min(MAX_FUEL_CAPACITY, estimatedHeldBalls + 1);
    }

    public void decrementBallCount() {
        estimatedHeldBalls = Math.max(0, estimatedHeldBalls - 1);
    }
}