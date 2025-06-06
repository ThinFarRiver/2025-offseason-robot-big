package frc.robot.subsystems.superstructure;

import lombok.Getter;
import lombok.RequiredArgsConstructor;
import frc.robot.subsystems.superstructure.SuperstructurePose.Preset;
import frc.robot.RobotConstants.IntakeConstants;
import frc.robot.RobotConstants.EndEffectorArmConstants;
import java.util.function.DoubleSupplier;

@Getter
@RequiredArgsConstructor
public enum SuperstructureState {
    // Stow positions
    START(createState(Preset.START)),
    CORAL_STOW(createState(Preset.CORAL_STOW)),
    ALGAE_STOW(createState(Preset.ALGAE_STOW)),
    IDLE(createState(Preset.IDLE)),
    AVOID(createState(Preset.AVOID)),

    // L1 positions
    L1_INTAKE_SIDE(createState(Preset.L1_INTAKE_SIDE)),
    L1_INTAKE_SIDE_EJECT(createEEState(L1_INTAKE_SIDE, () -> IntakeConstants.OUTTAKE_VOLTAGE.get())),
    L1_SHOOT_SIDE(createState(Preset.L1_SHOOT_SIDE)),
    L1_SHOOT_SIDE_EJECT(createEEState(L1_SHOOT_SIDE, () -> EndEffectorArmConstants.CORAL_SHOOT_VOLTAGE_L1.get())),

    // L2-L4 positions
    L2(createState(Preset.L2)),
    L2_EJECT(createEEState(L2, () -> EndEffectorArmConstants.CORAL_SHOOT_VOLTAGE.get())),
    L3(createState(Preset.L3)),
    L3_EJECT(createEEState(L3, () -> EndEffectorArmConstants.CORAL_SHOOT_VOLTAGE.get())),
    L4(createState(Preset.L4)),
    L4_EJECT(createEEState(L4, () -> EndEffectorArmConstants.CORAL_SHOOT_VOLTAGE.get())),

    // Net scoring positions
    NET_SCORE(createState(Preset.NET_SCORE)),
    NET_SCORE_EJECT(createEEState(NET_SCORE, () -> EndEffectorArmConstants.ALGAE_NET_SHOOT_VOLTAGE.get())),

    // Pickup positions
    P1(createEEState(Preset.P1, () -> EndEffectorArmConstants.ALGAE_INTAKE_VOLTAGE.get())),
    P2(createEEState(Preset.P2, () -> EndEffectorArmConstants.ALGAE_INTAKE_VOLTAGE.get())),
    CORAL_GROUND_INTAKE(createIntakeState(Preset.CORAL_GROUND_INTAKE));

    private final SuperstructureStateData value;

    // Helper methods to create states
    //TODO clean up helper states
    private static SuperstructureStateData createState(Preset preset) {
        return SuperstructureStateData.builder()
            .pose(preset.getPose())
            .build();
    }

    private static SuperstructureStateData createEEState(SuperstructureState baseState, DoubleSupplier voltage) {
        return baseState.getValue().toBuilder()
            .endEffectorVolts(voltage)
            .build();
    }

    private static SuperstructureStateData createEEState(Preset preset, DoubleSupplier voltage) {
        return SuperstructureStateData.builder()
            .pose(preset.getPose())
            .endEffectorVolts(voltage)
            .build();
    }

    private static SuperstructureStateData createIntakeState(Preset preset) {
        return SuperstructureStateData.builder()
            .pose(preset.getPose())
            .intakeVolts(() -> IntakeConstants.INTAKE_VOLTAGE.get())
            .endEffectorVolts(() -> EndEffectorArmConstants.CORAL_INTAKE_VOLTAGE.get())
            .build();
    }

    private static SuperstructureStateData createFullState(Preset preset, DoubleSupplier intakeVoltage, DoubleSupplier eeVoltage) {
        return SuperstructureStateData.builder()
            .pose(preset.getPose())
            .intakeVolts(intakeVoltage)
            .endEffectorVolts(eeVoltage)
            .build();
    }
}