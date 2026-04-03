package org.team9140.frc2026.helpers;

import java.util.Optional;

import org.team9140.frc2026.FieldConstants;
import org.team9140.frc2026.Constants.Turret;
import org.team9140.lib.Util;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class AimAlign {
    private static InterpolatingDoubleTreeMap lookupMotorSpeedFromDistance = new InterpolatingDoubleTreeMap();
    private static InterpolatingDoubleTreeMap lookupAirtimeFromDistance = new InterpolatingDoubleTreeMap();

    static {
        lookupAirtimeFromDistance.put(2.0245, 0.8266666666666666667);
        lookupAirtimeFromDistance.put(2.3213, 1.0);
        lookupAirtimeFromDistance.put(2.66, 1.06333333333333333);
        lookupAirtimeFromDistance.put(2.91, 1.1666666666666667);
        lookupAirtimeFromDistance.put(3.19, 1.29);
        lookupAirtimeFromDistance.put(3.49, 1.41);
        lookupAirtimeFromDistance.put(3.81, 1.4316666666666667);
        lookupAirtimeFromDistance.put(4.17, 1.47333333333333);
        lookupAirtimeFromDistance.put(4.45, 1.6166666666666667);

        lookupMotorSpeedFromDistance.put(2.0245, 1900.0 / 60.0);
        lookupMotorSpeedFromDistance.put(2.3213, 2050.0 / 60.0);
        lookupMotorSpeedFromDistance.put(2.66, 2150.0 / 60.0);
        lookupMotorSpeedFromDistance.put(2.91, 2300.0 / 60.0);
        lookupMotorSpeedFromDistance.put(3.19, 2425.0 / 60.0);
        lookupMotorSpeedFromDistance.put(3.49, 2575.0 / 60.0);
        lookupMotorSpeedFromDistance.put(3.81, 2700.0 / 60.0);
        lookupMotorSpeedFromDistance.put(4.17, 2825.0 / 60.0);
        lookupMotorSpeedFromDistance.put(4.45, 3050.0 / 60.0);
    }

    static StructPublisher<Pose2d> effectivePosePublisher = NetworkTableInstance.getDefault().getStructTopic("Effective Pose", Pose2d.struct).publish();

    public static Translation2d getEffectivePose(Pose2d robotPose, Translation2d goalPose, ChassisSpeeds robotSpeed) {
        Translation2d robotVelocity = new Translation2d(
                robotSpeed.vxMetersPerSecond,
                robotSpeed.vyMetersPerSecond);
        double distance, airtime;
        Translation2d newPose = goalPose.plus(robotVelocity.times(lookupAirtimeFromDistance.get(robotPose.plus(Turret.POSITION_TO_ROBOT).getTranslation().minus(goalPose).getNorm())));

        int iterations = 0;
        do {
            iterations++;
            distance = robotPose.plus(Turret.POSITION_TO_ROBOT).getTranslation().minus(newPose).getNorm();
            airtime = lookupAirtimeFromDistance.get(distance);
            SmartDashboard.putNumber("Estimated Airtime", airtime);
            effectivePosePublisher.set(new Pose2d(newPose, new Rotation2d()));
        } while (newPose.minus(newPose = goalPose.plus(robotVelocity.times(airtime))).getNorm() > 0.05 && iterations < 5);

        SmartDashboard.putNumber("NumIterations", iterations);

        return newPose;
    }

    public static double getRequiredSpeed(Pose2d robotPose, Translation2d effectivePose) {
        double distance = robotPose.plus(Turret.POSITION_TO_ROBOT).getTranslation().minus(effectivePose).getNorm();
        return lookupMotorSpeedFromDistance.get(distance);
    }

    public static double yawAngleToPos(Pose2d robotPose, Translation2d endPose) {
        endPose = (new Pose2d(endPose, Util.NOROTATION).relativeTo(robotPose)).getTranslation();
        return MathUtil.angleModulus(Math.atan2(
                (endPose.getY() - Turret.POSITION_TO_ROBOT.getY()),
                (endPose.getX() - Turret.POSITION_TO_ROBOT.getX()))
                - Turret.POSITION_TO_ROBOT.getRotation().getRadians());
    }

    public static Pose2d getZone(Pose2d robotPose) {
        double rx = robotPose.getX();
        double ry = robotPose.getY();
        Pose2d position;
        if (Optional.of(DriverStation.Alliance.Red).equals(Util.getAlliance())
                && rx > FieldConstants.Lines.RED_ALLIANCE_ZONE) {
            position = FieldConstants.Hub.RED_CENTER_POINT;
        } else if (Optional.of(DriverStation.Alliance.Blue).equals(Util.getAlliance())
                && rx < FieldConstants.Lines.BLUE_ALLIANCE_ZONE) {
            position = FieldConstants.Hub.CENTER_POINT;
        } else if (Optional.of(DriverStation.Alliance.Red).equals(Util.getAlliance())) {
            if (ry < FieldConstants.FIELD_WIDTH / 2) {
                position = FieldConstants.FeedingPositions.FEEDING_POS_LOWER_RED;
            } else {
                position = FieldConstants.FeedingPositions.FEEDING_POS_UPPER_RED;
            }
        } else {
            if (ry < FieldConstants.FIELD_WIDTH / 2) {
                position = FieldConstants.FeedingPositions.FEEDING_POS_LOWER;
            } else {
                position = FieldConstants.FeedingPositions.FEEDING_POS_UPPER;
            }
        }
        return position;
    }

    public static Pose2d getHub() {
        if (Util.getAlliance().equals(Optional.of(DriverStation.Alliance.Red))) {
            return FieldConstants.Hub.RED_CENTER_POINT;
        }

        return FieldConstants.Hub.CENTER_POINT;
    }
}
