package frc.robot.subsystems.superstructure.intake;

import org.littletonrobotics.junction.AutoLog;

import static frc.robot.RobotConstants.IntakeConstants.IntakePivotGainsClass.*;

public interface IntakePivotIO {
    default void updateInputs(IntakePivotIOInputs inputs) {
    }

    default void setMotorVoltage(double voltage) {
    }

    default void setPivotAngle(double targetAngleDeg) {
    }

    default void resetAngle(double resetAngleDeg) {
    }

    default boolean isNearAngle(double targetAngleDeg, double toleranceDeg) {
        return false;
    }

    @AutoLog
    class IntakePivotIOInputs {
        public double targetAngleDeg = 0.0;
        public double currentAngleDeg = 0.0;
        public double velocityRotPerSec = 0.0;
        public double appliedVolts = 0.0;
        public double motorVolts = 0.0;
        public double statorCurrentAmps = 0.0;
        public double supplyCurrentAmps = 0.0;
        public double tempCelsius = 0.0;

        public double intakePivotKP = INTAKE_PIVOT_KP.get();
        public double intakePivotKI = INTAKE_PIVOT_KI.get();
        public double intakePivotKD = INTAKE_PIVOT_KD.get();
        public double intakePivotKG = INTAKE_PIVOT_KG.get();
        public double intakePivotKS = INTAKE_PIVOT_KS.get();
    }
}