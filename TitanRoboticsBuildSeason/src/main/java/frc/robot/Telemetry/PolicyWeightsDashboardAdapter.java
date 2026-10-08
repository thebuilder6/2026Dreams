package frc.robot.Telemetry;

import org.littletonrobotics.junction.Logger;

import frc.robot.Data.Constants;
import frc.robot.Intelligence.PolicyWeights;

/**
 * Adapter that exposes key {@link PolicyWeights} to NetworkTables as {@link TunableNumber}s
 * when {@link Constants#TUNING_MODE} is active.
 *
 * <p>Enables live runtime tweaking of macro weights, dynamic action inertia,
 * and spatial fuel scent/clustering directly from Elastic Dashboard or AdvantageScope.
 */
public final class PolicyWeightsDashboardAdapter {
    private static final int CALLER_ID = 8334;

    // Macro weights
    private static final TunableNumber scoreHubBase = new TunableNumber("JevAI/ScoreHubBase", PolicyWeights.DEFAULT.scoreHubBase());
    private static final TunableNumber scoreHubScale = new TunableNumber("JevAI/ScoreHubScale", PolicyWeights.DEFAULT.scoreHubScale());
    private static final TunableNumber stageStandoffBase = new TunableNumber("JevAI/StageStandoffBase", PolicyWeights.DEFAULT.stageStandoffBase());
    private static final TunableNumber vacuumActiveBase = new TunableNumber("JevAI/VacuumActiveBase", PolicyWeights.DEFAULT.vacuumActiveBase());
    private static final TunableNumber vacuumInactiveBase = new TunableNumber("JevAI/VacuumInactiveBase", PolicyWeights.DEFAULT.vacuumInactiveBase());
    private static final TunableNumber sweepAllianceZoneActive = new TunableNumber("JevAI/SweepZoneActive", PolicyWeights.DEFAULT.sweepAllianceZoneActive());
    private static final TunableNumber laneDenialActiveUtility = new TunableNumber("JevAI/LaneDenialBase", PolicyWeights.DEFAULT.laneDenialActiveUtility());
    private static final TunableNumber shadowMidlineBaseUtility = new TunableNumber("JevAI/ShadowMidlineBase", PolicyWeights.DEFAULT.shadowMidlineBaseUtility());
    private static final TunableNumber interceptBaseUtility = new TunableNumber("JevAI/InterceptBase", PolicyWeights.DEFAULT.interceptBaseUtility());

    // Dynamic Action Inertia (Dave Mark)
    private static final TunableNumber commitmentMargin = new TunableNumber("JevAI/CommitmentMargin", PolicyWeights.DEFAULT.commitmentMargin());
    private static final TunableNumber commitmentDecisiveMargin = new TunableNumber("JevAI/CommitmentDecisiveMargin", PolicyWeights.DEFAULT.commitmentDecisiveMargin());
    private static final TunableNumber commitmentMinHoldSec = new TunableNumber("JevAI/CommitmentMinHoldSec", PolicyWeights.DEFAULT.commitmentMinHoldSec());
    private static final TunableNumber inertiaInitialBoost = new TunableNumber("JevAI/InertiaInitialBoost", PolicyWeights.DEFAULT.inertiaInitialBoost());
    private static final TunableNumber inertiaTimeConstantSec = new TunableNumber("JevAI/InertiaTimeConstantSec", PolicyWeights.DEFAULT.inertiaTimeConstantSec());
    private static final TunableNumber inertiaResidualMargin = new TunableNumber("JevAI/InertiaResidualMargin", PolicyWeights.DEFAULT.inertiaResidualMargin());

    // Spatial Fuel Scent / Clustering (EQS)
    private static final TunableNumber clusterNeighborhoodRadius = new TunableNumber("JevAI/ClusterRadius", PolicyWeights.DEFAULT.clusterNeighborhoodRadius());
    private static final TunableNumber clusterKernelSigma = new TunableNumber("JevAI/ClusterKernelSigma", PolicyWeights.DEFAULT.clusterKernelSigma());
    private static final TunableNumber clusterDensityExponent = new TunableNumber("JevAI/ClusterDensityExponent", PolicyWeights.DEFAULT.clusterDensityExponent());
    private static final TunableNumber clusterDistanceFloor = new TunableNumber("JevAI/ClusterDistanceFloor", PolicyWeights.DEFAULT.clusterDistanceFloor());
    private static final TunableNumber harvestHeadingAlignScale = new TunableNumber("JevAI/HarvestHeadingAlignScale", PolicyWeights.DEFAULT.harvestHeadingAlignScale());
    private static final TunableNumber harvestReturnVectorBonus = new TunableNumber("JevAI/HarvestReturnVectorBonus", PolicyWeights.DEFAULT.harvestReturnVectorBonus());

    // IAUS Consideration Curve Shapes (plan §4)
    private static final TunableNumber scoreHubPayloadExponent = new TunableNumber("JevAI/ScoreHubPayloadExponent", PolicyWeights.DEFAULT.scoreHubPayloadExponent());
    private static final TunableNumber shiftUrgencySigmoidSteepness = new TunableNumber("JevAI/ShiftUrgencySteepness", PolicyWeights.DEFAULT.shiftUrgencySigmoidSteepness());
    private static final TunableNumber shiftUrgencyMidpointSec = new TunableNumber("JevAI/ShiftUrgencyMidpointSec", PolicyWeights.DEFAULT.shiftUrgencyMidpointSec());
    private static final TunableNumber optimalStandoffMidpointM = new TunableNumber("JevAI/StandoffMidpointM", PolicyWeights.DEFAULT.optimalStandoffMidpointM());
    private static final TunableNumber optimalStandoffSigmaM = new TunableNumber("JevAI/StandoffSigmaM", PolicyWeights.DEFAULT.optimalStandoffSigmaM());

    private PolicyWeightsDashboardAdapter() {}

    /**
     * Checks all tunable numbers and applies updates to {@link PolicyWeights#setActive(PolicyWeights)}
     * if any value has changed on the dashboard.
     */
    public static void update() {
        if (!Constants.TUNING_MODE) {
            return;
        }

        boolean changed = scoreHubBase.hasChanged(CALLER_ID)
                || scoreHubScale.hasChanged(CALLER_ID)
                || stageStandoffBase.hasChanged(CALLER_ID)
                || vacuumActiveBase.hasChanged(CALLER_ID)
                || vacuumInactiveBase.hasChanged(CALLER_ID)
                || sweepAllianceZoneActive.hasChanged(CALLER_ID)
                || laneDenialActiveUtility.hasChanged(CALLER_ID)
                || shadowMidlineBaseUtility.hasChanged(CALLER_ID)
                || interceptBaseUtility.hasChanged(CALLER_ID)
                || commitmentMargin.hasChanged(CALLER_ID)
                || commitmentDecisiveMargin.hasChanged(CALLER_ID)
                || commitmentMinHoldSec.hasChanged(CALLER_ID)
                || inertiaInitialBoost.hasChanged(CALLER_ID)
                || inertiaTimeConstantSec.hasChanged(CALLER_ID)
                || inertiaResidualMargin.hasChanged(CALLER_ID)
                || clusterNeighborhoodRadius.hasChanged(CALLER_ID)
                || clusterKernelSigma.hasChanged(CALLER_ID)
                || clusterDensityExponent.hasChanged(CALLER_ID)
                || clusterDistanceFloor.hasChanged(CALLER_ID)
                || harvestHeadingAlignScale.hasChanged(CALLER_ID)
                || harvestReturnVectorBonus.hasChanged(CALLER_ID)
                || scoreHubPayloadExponent.hasChanged(CALLER_ID)
                || shiftUrgencySigmoidSteepness.hasChanged(CALLER_ID)
                || shiftUrgencyMidpointSec.hasChanged(CALLER_ID)
                || optimalStandoffMidpointM.hasChanged(CALLER_ID)
                || optimalStandoffSigmaM.hasChanged(CALLER_ID);

        if (changed) {
            PolicyWeights updated = PolicyWeights.getActive().toBuilder()
                    .scoreHubBase(scoreHubBase.get())
                    .scoreHubScale(scoreHubScale.get())
                    .stageStandoffBase(stageStandoffBase.get())
                    .vacuumActiveBase(vacuumActiveBase.get())
                    .vacuumInactiveBase(vacuumInactiveBase.get())
                    .sweepAllianceZoneActive(sweepAllianceZoneActive.get())
                    .laneDenialActiveUtility(laneDenialActiveUtility.get())
                    .shadowMidlineBaseUtility(shadowMidlineBaseUtility.get())
                    .interceptBaseUtility(interceptBaseUtility.get())
                    .commitmentMargin(commitmentMargin.get())
                    .commitmentDecisiveMargin(commitmentDecisiveMargin.get())
                    .commitmentMinHoldSec(commitmentMinHoldSec.get())
                    .inertiaInitialBoost(inertiaInitialBoost.get())
                    .inertiaTimeConstantSec(inertiaTimeConstantSec.get())
                    .inertiaResidualMargin(inertiaResidualMargin.get())
                    .clusterNeighborhoodRadius(clusterNeighborhoodRadius.get())
                    .clusterKernelSigma(clusterKernelSigma.get())
                    .clusterDensityExponent(clusterDensityExponent.get())
                    .clusterDistanceFloor(clusterDistanceFloor.get())
                    .harvestHeadingAlignScale(harvestHeadingAlignScale.get())
                    .harvestReturnVectorBonus(harvestReturnVectorBonus.get())
                    .scoreHubPayloadExponent(scoreHubPayloadExponent.get())
                    .shiftUrgencySigmoidSteepness(shiftUrgencySigmoidSteepness.get())
                    .shiftUrgencyMidpointSec(shiftUrgencyMidpointSec.get())
                    .optimalStandoffMidpointM(optimalStandoffMidpointM.get())
                    .optimalStandoffSigmaM(optimalStandoffSigmaM.get())
                    .build();

            PolicyWeights.setActive(updated);
            Logger.recordOutput("JevAI/TuningUpdated", true);
        }
    }
}
