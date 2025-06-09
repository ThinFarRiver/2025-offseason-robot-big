package frc.robot.commands.aimSequences;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.FieldConstants;
import frc.robot.FieldConstants.Reef;
import frc.robot.RobotConstants;
import org.littletonrobotics.AllianceFlipUtil;
import org.littletonrobotics.junction.Logger;

import java.util.List;

public class AimGoalSupplier {
    private record TagCondition(int tagA, int tagB, char axis, int positiveResult, int negativeResult) {}

    /**
     * Calculates the optimal drive target position based on the robot's current position and goal position
     *
     * @param robot The current pose (position and rotation) of the robot
     * @param goal  The target pose to drive towards
     * @return A modified goal pose that accounts for optimal approach positioning
     */
    public static Pose2d getDriveTarget(Pose2d robot, Pose2d goal) {
        Transform2d offset = new Transform2d(goal, new Pose2d(robot.getTranslation(), goal.getRotation()));
        double yDistance = Math.abs(offset.getY());
        double xDistance = Math.abs(offset.getX());
        double shiftXT =
                MathUtil.clamp(
                        (yDistance / (Reef.faceLength * 2)) + ((xDistance - 0.3) / (Reef.faceLength * 3)),
                        0.0,
                        1.0);
        double shiftYT = MathUtil.clamp(yDistance <= 0.2 ? 0.0 : -offset.getX() / Reef.faceLength, 0.0, 1.0);
        goal = goal.transformBy(
                new Transform2d(
                        shiftXT * RobotConstants.ReefAimConstants.MAX_DISTANCE_REEF_LINEUP.get(),
                        Math.copySign(shiftYT * RobotConstants.ReefAimConstants.MAX_DISTANCE_REEF_LINEUP.get() * 0.8, offset.getY()),
                        new Rotation2d()));

        return goal;
    }

    /**
     * Calculates the final target position for coral scoring based on the tag pose
     *
     * @param goal      The initial goal pose
     * @param rightReef Whether to target the right reef relative to the AprilTag
     * @return Modified goal pose to tag pose accounting for coral scoring position
     */
    public static Pose2d getFinalCoralTarget(Pose2d goal, boolean rightReef) {
        goal = goal.transformBy(new Transform2d(
                new Translation2d(
                        RobotConstants.ReefAimConstants.ROBOT_TO_PIPE_METERS.get(),
                        RobotConstants.ReefAimConstants.PIPE_TO_TAG.magnitude() * (rightReef ? 1 : -1)),
                new Rotation2d()));
        return goal;
    }

    /**
     * Calculates the final target position for algae scoring based on the tag pose
     *
     * @param goal The initial goal pose
     * @return Modified goal pose to tag pose accounting for algae scoring position
     */
    public static Pose2d getFinalAlgaeTarget(Pose2d goal) {
        goal = goal.transformBy(new Transform2d(
                new Translation2d(
                        RobotConstants.ReefAimConstants.ROBOT_TO_ALGAE_METERS.get(),
                        RobotConstants.ReefAimConstants.ALGAE_TO_TAG_METERS.get()),
                new Rotation2d()));
        return goal;
    }

    /**
     * Gets the nearest AprilTag pose to the robot's current position
     *
     * @param robotPose Current pose of the robot
     * @return Pose2d of the nearest AprilTag, accounting for edge cases and controller input
     */
    public static Pose2d getNearestTag(Pose2d robotPose) {
        return FieldConstants.officialAprilTagType.getLayout().getTagPose(getNearestTagID(robotPose)).get().toPose2d();
    }

    /**
     * Gets the ID of the nearest AprilTag to the robot's current position
     *
     * @param robotPose Current pose of the robot
     * @return ID of the nearest AprilTag, accounting for edge cases and controller input
     */
    public static int getNearestTagID(Pose2d robotPose) {
        XboxController driverController = new XboxController(0);
        double ControllerX = driverController.getLeftX();
        double ControllerY = driverController.getLeftY();
        double minDistance = Double.MAX_VALUE;
        double secondMinDistance = Double.MAX_VALUE;
        int ReefTagMin = AllianceFlipUtil.shouldFlip() ? 6 : 17;
        int ReefTagMax = AllianceFlipUtil.shouldFlip() ? 11 : 22;
        int minDistanceID = ReefTagMin;
        int secondMinDistanceID = ReefTagMin;
        for (int i = ReefTagMin; i <= ReefTagMax; i++) {
            double distance = FieldConstants.officialAprilTagType.getLayout().getTagPose(i).get().
                    toPose2d().getTranslation().getDistance(robotPose.getTranslation());
            if (distance < secondMinDistance) {
                secondMinDistanceID = i;
                secondMinDistance = distance;
            }
            if (distance < minDistance) {
                secondMinDistanceID = minDistanceID;
                secondMinDistance = minDistance;
                minDistanceID = i;
                minDistance = distance;
            }
        }
        if ((secondMinDistance - minDistance) < RobotConstants.ReefAimConstants.Edge_Case_Max_Delta.get() && (Math.abs(ControllerX) >= 0.05 || Math.abs(ControllerY) >= 0.05)) {
            minDistanceID = solveEdgeCase(ControllerX, ControllerY, minDistanceID, secondMinDistanceID);
        }
        return minDistanceID;
    }

    public static int solveEdgeCase(double controllerX, double controllerY, int minDistanceID, int secondMinDistanceID) {
        List<TagCondition> conditions = AllianceFlipUtil.shouldFlip() ?
                List.of(
                        new TagCondition(6, 11, 'Y', 6, 11),
                        new TagCondition(8, 9, 'Y', 8, 9),
                        new TagCondition(6, 7, 'X', 7, 6),
                        new TagCondition(7, 8, 'X', 8, 7),
                        new TagCondition(9, 10, 'X', 9, 10),
                        new TagCondition(10, 11, 'X', 10, 11)
                ) :
                List.of(
                        new TagCondition(20, 19, 'Y', 19, 20),
                        new TagCondition(17, 22, 'Y', 17, 22),
                        new TagCondition(17, 18, 'X', 17, 18),
                        new TagCondition(18, 19, 'X', 18, 19),
                        new TagCondition(21, 22, 'X', 22, 21),
                        new TagCondition(20, 21, 'X', 21, 20)
                );
        for (TagCondition condition : conditions) {
            if (correctTagPair(secondMinDistanceID, minDistanceID, condition.tagA(), condition.tagB())) {
                double value = condition.axis() == 'X' ? controllerX : controllerY;
                minDistanceID = value > 0 ? condition.positiveResult() : condition.negativeResult();
                break;
            }
        }
        return minDistanceID;
    }

    private static boolean correctTagPair(double tag1, double tag2, double wantedTag1, double wantedTag2) {
        return (tag1 == wantedTag1 && tag2 == wantedTag2) || (tag1 == wantedTag2 && tag2 == wantedTag1);
    }

    /**
     * Checks if the robot is in an edge case situation between two tags and logs relevant information
     *
     * @param robotPose Current pose of the robot
     */
    public static void isEdgeCase(Pose2d robotPose) {
        XboxController driverController = new XboxController(0);
        double ControllerX = driverController.getLeftX();
        double ControllerY = driverController.getLeftY();
        double minDistance = Double.MAX_VALUE;
        double secondMinDistance = Double.MAX_VALUE;
        int ReefTagMin = AllianceFlipUtil.shouldFlip() ? 6 : 17;
        int ReefTagMax = AllianceFlipUtil.shouldFlip() ? 11 : 22;
        int minDistanceID = ReefTagMin;
        int secondMinDistanceID = ReefTagMin;
        for (int i = ReefTagMin; i <= ReefTagMax; i++) {
            double distance = FieldConstants.officialAprilTagType.getLayout().getTagPose(i).get().
                    toPose2d().getTranslation().getDistance(robotPose.getTranslation());
            if (distance < secondMinDistance) {
                secondMinDistanceID = i;
                secondMinDistance = distance;
            }
            if (distance < minDistance) {
                secondMinDistanceID = minDistanceID;
                secondMinDistance = minDistance;
                minDistanceID = i;
                minDistance = distance;
            }
        }
        Logger.recordOutput("EdgeCase/DeltaDistance", secondMinDistance - minDistance);
        Logger.recordOutput("EdgeCase/ControllerX", ControllerX);
        Logger.recordOutput("EdgeCase/ControllerY", ControllerY);
        if ((secondMinDistance - minDistance) < RobotConstants.ReefAimConstants.Edge_Case_Max_Delta.get()) {
            Logger.recordOutput("EdgeCase/IsEdgeCase", true);
            if (Math.abs(ControllerX) >= 0.05 || Math.abs(ControllerY) >= 0.05) {
                minDistanceID = solveEdgeCase(ControllerX, ControllerY, minDistanceID, secondMinDistanceID);
            }
        } else {
            Logger.recordOutput("EdgeCase/IsEdgeCase", false);
        }
        Logger.recordOutput("EdgeCase/TargetChanged", minDistanceID == secondMinDistanceID);
    }
} 