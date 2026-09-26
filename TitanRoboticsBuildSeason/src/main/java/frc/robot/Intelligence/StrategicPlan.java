package frc.robot.Intelligence;

/** A current objective paired with its planned follow-up and transition time. */
public record StrategicPlan(
        StrategicObjective currentObjective,
        StrategicObjective nextObjective,
        double timeToTransitionSec) {
}
