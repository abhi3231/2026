package org.team9140.frc2026.helpers;

import java.util.Optional;

import org.team9140.frc2026.FieldConstants;
import org.team9140.frc2026.Constants.Turret;
import org.team9140.lib.Util;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;

public class AimAlign {
    private static InterpolatingDoubleTreeMap lookupMotorSpeedFromDistance = new InterpolatingDoubleTreeMap();
    private static InterpolatingDoubleTreeMap lookupAirtimeFromDistance = new InterpolatingDoubleTreeMap();

    static {
        lookupMotorSpeedFromDistance.put(1.973, 2000.0 / 60);
        lookupMotorSpeedFromDistance.put(2.59, 2250.0 / 60);
        lookupMotorSpeedFromDistance.put(3.167, 2500 / 60.0);
        lookupMotorSpeedFromDistance.put(3.545, 2650 / 60.0);
        lookupMotorSpeedFromDistance.put(4.193, 2750 / 60.0);
        lookupMotorSpeedFromDistance.put(4.333, 2900 / 60.0);
        lookupMotorSpeedFromDistance.put(5.449, 3150 / 60.0);


        lookupAirtimeFromDistance.put(Double.valueOf(14.85), Double.valueOf(0.2));
        lookupAirtimeFromDistance.put(Double.valueOf(9.74), Double.valueOf(-0.119));
    }

    public static Translation2d getEffectivePose(Pose2d robotPose, Translation2d goalPose, ChassisSpeeds robotSpeed) {
        Translation2d robotVelocity = new Translation2d(
                robotSpeed.vxMetersPerSecond,
                robotSpeed.vyMetersPerSecond);
        Translation2d oldPose = goalPose.minus(robotPose.plus(Turret.POSITION_TO_ROBOT).getTranslation());
        double airtime = lookupAirtimeFromDistance.get(oldPose.getNorm());
        Translation2d newPose = goalPose.minus(robotVelocity.times(airtime));
        while (oldPose.minus(newPose).getNorm() > 0.1) {
            oldPose = newPose;
            airtime = lookupAirtimeFromDistance.get(oldPose.getNorm());
            newPose = goalPose.minus(robotVelocity.times(airtime));
        }
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
}
