package frc.robot.subsystems.superstructure;

import lombok.Getter;
import lombok.RequiredArgsConstructor;
import frc.robot.subsystems.superstructure.SuperstructurePose.Preset;
import frc.robot.RobotConstants.IntakeConstants;
import frc.robot.RobotConstants.EndEffectorArmConstants;

@Getter
@RequiredArgsConstructor
public enum SuperstructureState {
    START(SuperstructureStateData.builder()
        .pose(Preset.START.getPose())
        .build()),
    CORAL_STOW(SuperstructureStateData.builder()
        .pose(Preset.CORAL_STOW.getPose())
        .build()),
    ALGAE_STOW(SuperstructureStateData.builder()
        .pose(Preset.ALGAE_STOW.getPose())
        .build()),
    L1_INTAKE_SIDE(SuperstructureStateData.builder()
        .pose(Preset.L1_INTAKE_SIDE.getPose())
        .build()),
    L1_INTAKE_SIDE_EJECT(L1_INTAKE_SIDE.getValue().toBuilder()
        .intakeVolts(() -> IntakeConstants.OUTTAKE_VOLTAGE.get())
        .build()),
    L1_SHOOT_SIDE(SuperstructureStateData.builder()
        .pose(Preset.L1_SHOOT_SIDE.getPose())
        .build()),
    L1_SHOOT_SIDE_EJECT(L1_SHOOT_SIDE.getValue().toBuilder()
        .intakeVolts(() -> EndEffectorArmConstants.CORAL_SHOOT_VOLTAGE_L1.get())
        .build()),
    L2(SuperstructureStateData.builder()
        .pose(Preset.L2.getPose())
        .build()),
    L2_EJECT(L2.getValue().toBuilder()
        .endEffectorVolts(() -> EndEffectorArmConstants.CORAL_SHOOT_VOLTAGE.get())
        .build()),
    L3(SuperstructureStateData.builder()
        .pose(Preset.L3.getPose())
        .build()),
    L3_EJECT(L3.getValue().toBuilder()
        .endEffectorVolts(() -> EndEffectorArmConstants.CORAL_SHOOT_VOLTAGE.get())
        .build()),
    L4(SuperstructureStateData.builder()
        .pose(Preset.L4.getPose())
        .build()),
    L4_EJECT(L4.getValue().toBuilder()
        .endEffectorVolts(() -> EndEffectorArmConstants.CORAL_SHOOT_VOLTAGE.get())
        .build()),
    NET_SCORE(SuperstructureStateData.builder()
        .pose(Preset.NET_SCORE.getPose())
        .build()),
    NET_SCORE_EJECT(NET_SCORE.getValue().toBuilder()
        .endEffectorVolts(() -> EndEffectorArmConstants.ALGAE_NET_SHOOT_VOLTAGE.get())
        .build()),
    P1(SuperstructureStateData.builder()
        .pose(Preset.P1.getPose())
        .endEffectorVolts(() -> EndEffectorArmConstants.ALGAE_INTAKE_VOLTAGE.get())
        .build()),
    P2(SuperstructureStateData.builder()
        .pose(Preset.P2.getPose())
        .endEffectorVolts(() -> EndEffectorArmConstants.ALGAE_INTAKE_VOLTAGE.get())
        .build()),
    CORAL_GROUND_INTAKE(
        SuperstructureStateData.builder()
        .pose(Preset.CORAL_GROUND_INTAKE.getPose())
        .intakeVolts(() -> IntakeConstants.INTAKE_VOLTAGE.get())
        .build()),
    IDLE(SuperstructureStateData.builder()
        .pose(Preset.IDLE.getPose())
        .build()),
    AVOID(SuperstructureStateData.builder()
        .pose(Preset.AVOID.getPose())
        .build());

    private final SuperstructureStateData value;
} 