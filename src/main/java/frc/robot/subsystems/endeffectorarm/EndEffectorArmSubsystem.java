package frc.robot.subsystems.endeffectorarm;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.RobotConstants;
import frc.robot.RobotContainer;
import frc.robot.drivers.DestinationSupplier;
import frc.robot.drivers.GamepieceTracker;
import frc.robot.subsystems.beambreak.BeambreakIO;
import frc.robot.subsystems.beambreak.BeambreakIOInputsAutoLogged;
import frc.robot.subsystems.roller.RollerIOInputsAutoLogged;
import frc.robot.subsystems.roller.RollerSubsystem;
import frc.robot.subsystems.superstructure.SuperstructureVisualizer;
import frc.robot.utils.TimeDelayedBoolean;
import lombok.Getter;
import lombok.Setter;
import org.littletonrobotics.junction.Logger;

import static frc.robot.RobotConstants.EndEffectorArmConstants.*;

public class EndEffectorArmSubsystem extends RollerSubsystem {
    public static final String NAME = "EndEffectorArm";
    // Static variables to hold the current values from TunableNumbers
    private static double homeAngle = HOME_ANGLE.get();
    private static double coralIntakeAngle = CORAL_INTAKE_ANGLE.get();
    private static double coralOuttakeAngle = CORAL_OUTTAKE_ANGLE.get();
    private static double coralPreShootAngle = CORAL_PRESHOOT_ANGLE.get();
    private static double coralPreShootAngleL4 = CORAL_PRESHOOT_ANGLE_L4.get();
    private static double coralPreShootAngleL1 = CORAL_PRESHOOT_ANGLE_L1.get();

    private static double algaeIntakeAngle = ALGAE_INTAKE_ANGLE.get();
    private static double algaeNetPreShootAngle = ALGAE_NET_PRESHOOT_ANGLE.get();
    private static double algaeProcessorPreShootAngle = ALGAE_PROCESSOR_PRESHOOT_ANGLE.get();
    private static double algaeProcessorShootVoltage = ALGAE_PROCESSOR_SHOOT_VOLTAGE.get();
    private static double coralIntakeVoltage = CORAL_INTAKE_VOLTAGE.get();
    private static double coralOuttakeVoltage = CORAL_OUTTAKE_VOLTAGE.get();
    private static double coralShootVoltageL1 = CORAL_SHOOT_VOLTAGE_L1.get();
    private static double algaeIntakeVoltage = ALGAE_INTAKE_VOLTAGE.get();

    private static double coralHoldVoltage = CORAL_HOLD_VOLTAGE.get();
    private static double algaeHoldVoltage = ALGAE_HOLD_VOLTAGE.get();
    // Add these constants near the other static variables
    private static double coralShootVoltage = CORAL_SHOOT_VOLTAGE.get();
    private static double algaeNetShootVoltage = ALGAE_NET_SHOOT_VOLTAGE.get();
    private final Timer algaeShootTimer = new Timer();
    // IO devices and their inputs
    private final EndEffectorArmPivotIO armPivotIO;
    private final EndEffectorArmRollerIO armRollerIO;
    private final EndEffectorArmPivotIOInputsAutoLogged armPivotIOInputs = new EndEffectorArmPivotIOInputsAutoLogged();
    private final RollerIOInputsAutoLogged armRollerIOInputs = new RollerIOInputsAutoLogged();
    // Beambreak sensors for coral and algae detection
    private final BeambreakIO coralBeambreakIO;
    private final BeambreakIO algaeBeambreakIO;
    private final BeambreakIOInputsAutoLogged coralBeambreakInputs = new BeambreakIOInputsAutoLogged();
    private final BeambreakIOInputsAutoLogged algaeBeambreakInputs = new BeambreakIOInputsAutoLogged();
    private final Timer simGamepieceTimer = new Timer();
    private boolean algaeShootTimerStarted = false;
    // State tracking
    @Setter
    private WantedState wantedState = WantedState.HOLD;
    @Getter
    private SystemState systemState = SystemState.HOLDING;

    private Timer isCoralShootingStartedTimer = new Timer();


    /**
     * Creates a new EndEffectorArmSubsystem
     *
     * @param armPivotIO       The IO interface for the arm pivot
     * @param armRollerIO      The IO interface for the roller
     * @param coralBeambreakIO The beambreak sensor for coral detection
     * @param algaeBeambreakIO The beambreak sensor for algae detection
     */
    public EndEffectorArmSubsystem(
            EndEffectorArmPivotIO armPivotIO,
            EndEffectorArmRollerIO armRollerIO,
            BeambreakIO coralBeambreakIO,
            BeambreakIO algaeBeambreakIO) {
        super(armRollerIO, NAME);
        this.armPivotIO = armPivotIO;
        this.armRollerIO = armRollerIO;
        this.coralBeambreakIO = coralBeambreakIO;
        this.algaeBeambreakIO = algaeBeambreakIO;
    }

    @Override
    public void periodic() {
        super.periodic();

        // Update inputs from hardware
        armPivotIO.updateInputs(armPivotIOInputs);
        coralBeambreakIO.updateInputs(coralBeambreakInputs);
        algaeBeambreakIO.updateInputs(algaeBeambreakInputs);

        if (RobotBase.isReal()) {
            // If the robot is real, Update DestinationSupplier with current coral and algae states
            GamepieceTracker.getInstance().setEndeffectorHasCoral(coralBeambreakInputs.isBeambreakOn);
            GamepieceTracker.getInstance().setEndeffectorHasAlgae(algaeBeambreakInputs.isBeambreakOn);
        } else {
            //sim gamepiece tracking
            if (systemState == SystemState.CORAL_INTAKING && GamepieceTracker.getInstance().isIntakeHasCoral() && isNearAngle(coralIntakeAngle)) {
                GamepieceTracker.getInstance().setEndeffectorHasCoral(true);
                GamepieceTracker.getInstance().setintakeHasCoral(false);
            }
            if (systemState == SystemState.CORAL_SHOOTING) {
                GamepieceTracker.getInstance().setEndeffectorHasCoral(false);
            }
            if (systemState == SystemState.ALGAE_INTAKING && isNearAngle(algaeIntakeAngle)) {
                GamepieceTracker.getInstance().setEndeffectorHasAlgae(true);
            }
            if (systemState == SystemState.ALGAE_NET_SHOOTING || systemState == SystemState.ALGAE_PROCESSOR_SHOOTING) {
                GamepieceTracker.getInstance().setEndeffectorHasAlgae(false);
            }
        }

        // Update danger flag based on arm position
        RobotContainer.endeffectorIsDanger = !(
                isNearAngle(coralIntakeAngle) ||
                        (isNearAngle(coralPreShootAngle) && hasCoral()) ||
                        (isNearAngle(coralPreShootAngleL1) && hasCoral()) ||
                        (isNearAngle(coralPreShootAngleL4) && hasCoral()) ||
                        (isNearAngle(algaeProcessorPreShootAngle, 10) && hasAlgae())
        );

        // Calculate current state transition
        SystemState newState;
        if (RobotContainer.elevatorIsDanger) {
            // Force specific states while elevator is in danger
            newState = switch (wantedState) {
                case CORAL_OUTTAKE -> SystemState.CORAL_OUTTAKING;
                case CORAL_INTAKE -> SystemState.CORAL_INTAKING;
                case CORAL_SHOOT -> SystemState.CORAL_SHOOTING;
                case CORAL_PRESHOOT -> SystemState.CORAL_PRESHOOTING;
                case ALGAE_PROCESSOR_PRESHOOT -> SystemState.ALGAE_PROCESSOR_PRESHOOTING;
                case ALGAE_PROCESSOR_SHOOT -> SystemState.ALGAE_PROCESSOR_SHOOTING;
                case ALGAE_INTAKE -> SystemState.ALGAE_INTAKING;
                default -> systemState;
            };
        } else {
            // Normal state transitions when elevator is safe
            newState = handleStateTransition();
        }

        // Process and log inputs
        Logger.processInputs(NAME + "/Pivot", armPivotIOInputs);
        Logger.processInputs(NAME + "/Coral Beambreak", coralBeambreakInputs);
        Logger.processInputs(NAME + "/Algae Beambreak", algaeBeambreakInputs);

        // Log the system state
        Logger.recordOutput("EndEffectorArm/SystemState", systemState.toString());

        Logger.recordOutput("Flags/eeIsDanger", RobotContainer.endeffectorIsDanger);
        Logger.recordOutput("Flags/eeIsDangerOverride", RobotContainer.overrideEndEffectorDanger);

        SuperstructureVisualizer.getInstance().updateEndEffector(armPivotIOInputs.currentAngleDeg);

        // Apply the new state if it has changed
        if (newState != systemState) {
            systemState = newState;
        }

        // Handle actions based on current system state
        switch (systemState) {
            case CORAL_INTAKING:
                armRollerIO.setVoltage(coralIntakeVoltage);
                armPivotIO.setPivotAngle(coralIntakeAngle);
                break;

            case CORAL_OUTTAKING:
                armRollerIO.setVoltage(coralOuttakeVoltage);
                armPivotIO.setPivotAngle(coralOuttakeAngle);
                break;

            case CORAL_PRESHOOTING:
                if (DestinationSupplier.getInstance().getCurrentElevSetpointCoral() == DestinationSupplier.elevatorSetpoint.L1) {
                    armPivotIO.setPivotAngle(coralPreShootAngleL1);
                }
                else if(DestinationSupplier.getInstance().getCurrentElevSetpointCoral() == DestinationSupplier.elevatorSetpoint.L4){
                    armPivotIO.setPivotAngle(coralPreShootAngleL4);
                }
                else {
                    armPivotIO.setPivotAngle(coralPreShootAngle);
                }
                break;
            case ALGAE_INTAKING:
                armRollerIO.setVoltage(algaeIntakeVoltage);
                armPivotIO.setPivotAngle(algaeIntakeAngle);
                break;

            case ALGAE_NET_PRESHOOTING:
                armPivotIO.setPivotAngle(algaeNetPreShootAngle);
                break;

            case ALGAE_NET_SHOOTING:
                armRollerIO.setVoltage(algaeNetShootVoltage);
                break;

            case ALGAE_PROCESSOR_PRESHOOTING:
                armPivotIO.setPivotAngle(algaeProcessorPreShootAngle);
                break;

            case ALGAE_PROCESSOR_SHOOTING:
                armRollerIO.setVoltage(algaeProcessorShootVoltage);
                break;

            case HOLDING:
                //TODO: cant allow both algae and coral to be held at the same time
                if (hasAlgae()) {
                    armRollerIO.setVoltage(algaeHoldVoltage);
                    armPivotIO.setPivotAngle(coralIntakeAngle);
                } else if (hasCoral()) {
                    armPivotIO.setPivotAngle(homeAngle);
                    armRollerIO.setVoltage(coralHoldVoltage);
                }
                break;

            case NEUTRAL:
                armRollerIO.stop();
                armPivotIO.setPivotAngle(coralIntakeAngle);
                break;

            case CORAL_SHOOTING:
                if (DestinationSupplier.getInstance().getCurrentElevSetpointCoral() == DestinationSupplier.elevatorSetpoint.L1) {
                    armRollerIO.setVoltage(coralShootVoltageL1);
                } else {
                    armRollerIO.setVoltage(coralShootVoltage);
                }
                break;
        }

        // Update tunable numbers if tuning is enabled
        if (RobotConstants.TUNING) {
            homeAngle = HOME_ANGLE.get();
            coralIntakeAngle = CORAL_INTAKE_ANGLE.get();
            coralOuttakeAngle = CORAL_OUTTAKE_ANGLE.get();
            coralPreShootAngle = CORAL_PRESHOOT_ANGLE.get();
            coralPreShootAngleL4 = CORAL_PRESHOOT_ANGLE_L4.get();
            coralPreShootAngleL1 = CORAL_PRESHOOT_ANGLE_L1.get();
            algaeIntakeAngle = ALGAE_INTAKE_ANGLE.get();
            algaeNetPreShootAngle = ALGAE_NET_PRESHOOT_ANGLE.get();
            algaeProcessorPreShootAngle = ALGAE_PROCESSOR_PRESHOOT_ANGLE.get();

            coralIntakeVoltage = CORAL_INTAKE_VOLTAGE.get();
            coralOuttakeVoltage = CORAL_OUTTAKE_VOLTAGE.get();
            coralShootVoltageL1 = CORAL_SHOOT_VOLTAGE_L1.get();
            algaeIntakeVoltage = ALGAE_INTAKE_VOLTAGE.get();
            coralHoldVoltage = CORAL_HOLD_VOLTAGE.get();
            algaeHoldVoltage = ALGAE_HOLD_VOLTAGE.get();

            coralShootVoltage = CORAL_SHOOT_VOLTAGE.get();
            algaeNetShootVoltage = ALGAE_NET_SHOOT_VOLTAGE.get();
            algaeProcessorShootVoltage = ALGAE_PROCESSOR_SHOOT_VOLTAGE.get();
        }
    }

    /**
     * Handles the transition between states based on the current wanted state
     *
     * @return The new system state
     */
    private SystemState handleStateTransition() {
        return switch (wantedState) {
            case CORAL_INTAKE -> {
                if (hasCoral()) {
                    setWantedState(WantedState.HOLD);
                    yield SystemState.HOLDING;
                }
                yield SystemState.CORAL_INTAKING;
            }
            case CORAL_OUTTAKE -> SystemState.CORAL_OUTTAKING;
            case CORAL_PRESHOOT -> {
                if (hasCoral()) {
                    yield SystemState.CORAL_PRESHOOTING;
                } else {
                    yield SystemState.HOLDING;
                }
            }
            case ALGAE_INTAKE -> {
                if (hasAlgae()) {
                    setWantedState(WantedState.HOLD);
                    yield SystemState.HOLDING;
                }
                yield SystemState.ALGAE_INTAKING;
            }
            case HOLD -> {
                if (!hasAlgae() && !hasCoral()) {
                    yield SystemState.NEUTRAL;
                }
                yield SystemState.HOLDING;
            }
            case NEUTRAL -> SystemState.NEUTRAL;
            case CORAL_SHOOT -> {
                if (!isShootFinished()) {
                    yield SystemState.CORAL_SHOOTING;
                } else {
                    if (!isCoralShootingStartedTimer.isRunning()) {
                        isCoralShootingStartedTimer.start();
                        yield SystemState.CORAL_SHOOTING;
                    } else if (isCoralShootingStartedTimer.hasElapsed(CORAL_SHOOT_DELAY_TIME.get())) {
                        isCoralShootingStartedTimer.stop();
                        isCoralShootingStartedTimer.reset();
                        setWantedState(WantedState.HOLD);
                        yield SystemState.HOLDING;
                    } else yield SystemState.CORAL_SHOOTING;
                }
            }
            case ALGAE_NET_SHOOT -> {
                if (hasAlgae()) {
                    yield SystemState.ALGAE_NET_SHOOTING;
                } else {
                    if (!algaeShootTimerStarted) {
                        algaeShootTimer.start();
                        algaeShootTimerStarted = true;
                        yield SystemState.ALGAE_NET_SHOOTING;
                    } else if (algaeShootTimer.hasElapsed(0.3)) {
                        algaeShootTimer.stop();
                        algaeShootTimer.reset();
                        algaeShootTimerStarted = false;
                        setWantedState(WantedState.HOLD);
                        yield SystemState.HOLDING;
                    } else yield SystemState.ALGAE_NET_SHOOTING;
                }

            }
            case ALGAE_NET_PRESHOOT -> {
                if (hasAlgae()) {
                    yield SystemState.ALGAE_NET_PRESHOOTING;
                } else {
                    yield SystemState.HOLDING;
                }
            }
            case ALGAE_PROCESSOR_SHOOT -> {
                if (hasAlgae()) {
                    yield SystemState.ALGAE_PROCESSOR_SHOOTING;
                } else {
                    if (!algaeShootTimerStarted) {
                        algaeShootTimer.start();
                        algaeShootTimerStarted = true;
                        yield SystemState.ALGAE_PROCESSOR_SHOOTING;
                    } else if (algaeShootTimer.hasElapsed(0.3)) {
                        algaeShootTimer.stop();
                        algaeShootTimer.reset();
                        algaeShootTimerStarted = false;
                        setWantedState(WantedState.HOLD);
                        yield SystemState.HOLDING;
                    } else yield SystemState.ALGAE_PROCESSOR_SHOOTING;
                }

            }
            case ALGAE_PROCESSOR_PRESHOOT -> {
                if (hasAlgae()) {
                    yield SystemState.ALGAE_PROCESSOR_PRESHOOTING;
                } else {
                    yield SystemState.HOLDING;
                }
            }
        };
    }

    /**
     * Checks if the arm is near a target angle
     *
     * @param targetAngleDeg The target angle in degrees
     * @return True if the arm is within 1 degree of the target
     */
    public boolean isNearAngle(double targetAngleDeg) {
        return MathUtil.isNear(targetAngleDeg, armPivotIOInputs.currentAngleDeg, 5);
        //TODO：isNear angle value
    }

    public boolean isNearAngle(double targetAngleDeg, double tolerance) {
        return MathUtil.isNear(targetAngleDeg, armPivotIOInputs.currentAngleDeg, tolerance);
        //TODO：isNear angle value
    }

    /**
     * Checks if the mechanism has coral
     *
     * @return True if coral is detected by the beambreak
     */
    public boolean hasCoral() {
        return GamepieceTracker.getInstance().isEndeffectorHasCoral();
    }

    /**
     * Checks if the mechanism has algae
     *
     * @return True if algae is detected by the beambreak
     */
    public boolean hasAlgae() {
        return GamepieceTracker.getInstance().isEndeffectorHasAlgae();
    }

    /**
     * Checks if the shoot is finished
     *
     * @return True if neither coral nor algae is detected
     */
    public boolean isShootFinished() {
        return !hasCoral() && !hasAlgae();
    }

    /**
     * Checks if is ready to shoot, i.e. angle is ready and has a coral
     *
     * @return True if both angle is near the preshoot angle and contains a coral
     */
    public boolean isShootReady() {
        if (DestinationSupplier.getInstance().getCurrentElevSetpointCoral() == DestinationSupplier.elevatorSetpoint.L1) {
            return isNearAngle(coralPreShootAngleL1);
        } else if(DestinationSupplier.getInstance().getCurrentElevSetpointCoral() == DestinationSupplier.elevatorSetpoint.L4){
            return isNearAngle(coralPreShootAngleL4);
        } else {
            return isNearAngle(coralPreShootAngle);
        }
    }

    /**
     * The wanted state for the EndEffectorArm subsystem
     */
    public enum WantedState {
        CORAL_INTAKE,
        CORAL_OUTTAKE,
        CORAL_PRESHOOT,
        CORAL_SHOOT,
        ALGAE_INTAKE,
        ALGAE_NET_PRESHOOT,
        ALGAE_NET_SHOOT,
        ALGAE_PROCESSOR_PRESHOOT,
        ALGAE_PROCESSOR_SHOOT,
        HOLD,
        NEUTRAL
    }

    /**
     * The current system state of the EndEffectorArm subsystem
     */
    public enum SystemState {
        CORAL_INTAKING,
        CORAL_OUTTAKING,
        CORAL_PRESHOOTING,
        CORAL_SHOOTING,
        ALGAE_INTAKING,
        ALGAE_NET_PRESHOOTING,
        ALGAE_NET_SHOOTING,
        ALGAE_PROCESSOR_PRESHOOTING,
        ALGAE_PROCESSOR_SHOOTING,
        HOLDING,
        NEUTRAL
    }
} 