package frc.robot.auto.routines;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.FieldConstants;
import frc.robot.auto.AutoActions;
import frc.robot.auto.AutoRoutine;
import frc.robot.commands.aimSequences.AimGoalSupplier;
import frc.robot.subsystems.superstructure.SuperstructureState;

import static edu.wpi.first.wpilibj2.command.Commands.*;
import static frc.robot.auto.AutoActions.*;

public class AutoLeft1C extends AutoRoutine {
  private static final Pose2d startPose = new Pose2d(
      new Translation2d(7.140, FieldConstants.fieldWidth - 0.50),
      Rotation2d.kZero
  );

  public AutoLeft1C() {
    super("Left1C");
  }

  @Override
  public Command getAutoCommand() {
    var scorePreload = sequence(
        setGoal(AimGoalSupplier.ReefFace.FarLeftTilt, false, SuperstructureState.L4),
        parallel(
            driveToSelectedTarget(),
            prepare()
        ),
        shoot()
    );

    var driveToDpAndIntake1 = deadline(
        sequence(
            sequence(
                driveToIntakePoint(true, true),
                driveForwardBlind(2.0, 1.0).until(AutoActions::isInIntakeDangerZone)
            ).until(AutoActions::isCoralInSight),
            chase().onlyIf(AutoActions::isCoralInSight)
        ),
        intake()
    );

    var ending = sequence(
        driveToEndPoint(true),
        indicateEnd()
    );


    return sequence(
        scorePreload,
        driveToDpAndIntake1,
        ending
    );
  }

  @Override
  public Command getOnSelectCommand() {
    return resetOnPose(startPose);
  }
}
