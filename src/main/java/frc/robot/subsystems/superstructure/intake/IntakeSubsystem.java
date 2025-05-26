package frc.robot.subsystems.superstructure.intake;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.RobotConstants;
import frc.robot.subsystems.beambreak.BeambreakIO;
import frc.robot.subsystems.beambreak.BeambreakIOInputsAutoLogged;
import frc.robot.subsystems.superstructure.intake.IntakePivotIOInputsAutoLogged;
import frc.robot.subsystems.roller.RollerIO;
import frc.robot.subsystems.roller.RollerIOInputsAutoLogged;
import frc.robot.subsystems.roller.RollerSubsystem;
import frc.robot.subsystems.superstructure.GamepieceTracker;
import frc.robot.subsystems.superstructure.SuperstructureVisualizer;

import java.util.function.DoubleSupplier;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;
import lombok.Getter;

public class IntakeSubsystem extends RollerSubsystem {
    private final IntakePivotIO intakePivotIO;
    private final RollerIO intakeRollerIO;
    private final IntakePivotIOInputsAutoLogged intakePivotIOInputs = new IntakePivotIOInputsAutoLogged();
    private final RollerIOInputsAutoLogged intakeRollerIOInputs = new RollerIOInputsAutoLogged();
    private final BeambreakIO BBIO;
    private final BeambreakIOInputsAutoLogged BBInputs = new BeambreakIOInputsAutoLogged();


    @Getter@AutoLogOutput(key = "Intake/setPoint")
    private double wantedAngle = 0.0;
    @Getter@AutoLogOutput(key = "Intake/atGoal")
    private boolean atGoal = false;

    public IntakeSubsystem(
            IntakePivotIO intakePivotIO,
            RollerIO intakeRollerIO,
            BeambreakIO BBIO
    ) {
        super(intakeRollerIO, "Intake/Roller");
        this.intakePivotIO = intakePivotIO;
        this.intakeRollerIO = intakeRollerIO;
        this.BBIO = BBIO;
    }

    @Override
    public void periodic() {
        super.periodic();

        BBIO.updateInputs(BBInputs);
        intakePivotIO.updateInputs(intakePivotIOInputs);
        intakeRollerIO.updateInputs(intakeRollerIOInputs);

        Logger.processInputs("Intake/Pivot", intakePivotIOInputs);
        Logger.processInputs("Intake/Roller", intakeRollerIOInputs);
        Logger.processInputs("Intake/Roller/Beambreak", BBInputs);
        atGoal = isNearAngle(wantedAngle);
        intakePivotIO.setPivotAngle(wantedAngle);

        if (RobotBase.isReal()) {
            GamepieceTracker.getInstance().setintakeHasCoral(BBInputs.isBeambreakOn);
        }
    }

    /**
     * Checks if the arm is near a target angle
     *
     * @param targetAngleDeg The target angle in degrees
     * @return True if the arm is within tolerance of the target
     */
    public boolean isNearAngle(double targetAngleDeg) {
        return MathUtil.isNear(targetAngleDeg, intakePivotIOInputs.currentAngleDeg, 1.0);
    }

    /**
     * Checks if the mechanism has coral
     *
     * @return True if coral is detected by the beambreak
     */
    public boolean hasCoral() {
        return GamepieceTracker.getInstance().isIntakeHasCoral();
    }

    /**
     * Checks if the intake is in a dangerous position
     *
     * @return True if the intake is in a dangerous position
     */
    public boolean intakeIsDanger() {
        return intakePivotIOInputs.currentAngleDeg < RobotConstants.IntakeConstants.INTAKE_DANGER_ZONE - 2;
    }

    // Basic control methods
    public void setPivotAngle(DoubleSupplier angleDeg) {
        wantedAngle = angleDeg.getAsDouble();
    }

    public void setPivotVoltage(double voltage) {
        intakePivotIO.setMotorVoltage(voltage);
    }

    public void setRollerVoltage(DoubleSupplier voltage) {
        intakeRollerIO.setVoltage(voltage.getAsDouble());
    }

    public void stopRoller() {
        intakeRollerIO.stop();
    }

    public double getCurrentAngle() {
        return intakePivotIOInputs.currentAngleDeg;
    }

}