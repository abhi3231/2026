package org.team9140.frc2026.helpers;

import java.util.Optional;

import org.team9140.frc2026.FieldConstants;
import org.team9140.lib.Util;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;

public class AimAlign {
    private static InterpolatingDoubleTreeMap lookupMotorSpeedFromDistance = new InterpolatingDoubleTreeMap();
    private static InterpolatingDoubleTreeMap lookupAirtimeFromDistance = new InterpolatingDoubleTreeMap();

    static {
        lookupMotorSpeedFromDistance.put(Double.valueOf(Units.feetToMeters(7)), 2250.0 / 60);
        lookupMotorSpeedFromDistance.put(Double.valueOf(Units.feetToMeters(8)), 2300.0 / 60);
        lookupMotorSpeedFromDistance.put(Double.valueOf(Units.feetToMeters(9)),  2400.0 / 60);
        lookupMotorSpeedFromDistance.put(Double.valueOf(Units.feetToMeters(10)), 2500.0 / 60);
        lookupMotorSpeedFromDistance.put(Double.valueOf(Units.feetToMeters(11)), 2600.0 / 60);
        lookupMotorSpeedFromDistance.put(Double.valueOf(Units.feetToMeters(12)), 2700.0 / 60);
        lookupMotorSpeedFromDistance.put(Double.valueOf(Units.feetToMeters(13)), 2825.0 / 60);

        lookupAirtimeFromDistance.put(Double.valueOf(14.85), Double.valueOf(0.2));
        lookupMotorSpeedFromDistance.put(Double.valueOf(9.74), Double.valueOf(-0.119));
    }

    public static Translation2d getEffectivePose(Pose2d turretPose, Translation2d goalPose, ChassisSpeeds robotSpeed) {
        Translation2d robotVelocity = new Translation2d(
                robotSpeed.vxMetersPerSecond,
                robotSpeed.vyMetersPerSecond);
        Translation2d oldPose = goalPose.minus(turretPose.getTranslation());
        double airtime = lookupAirtimeFromDistance.get(oldPose.getNorm());
        Translation2d newPose = goalPose.minus(robotVelocity.times(airtime));
        while (oldPose.minus(newPose).getNorm() > 0.1) {
            oldPose = newPose;
            airtime = lookupAirtimeFromDistance.get(oldPose.getNorm());
            newPose = goalPose.minus(robotVelocity.times(airtime));
        }
        return newPose;
    }

    public static double getRequiredSpeed(Pose2d turretPose, Translation2d effectivePose) {
        double distance = turretPose.getTranslation().minus(effectivePose).getNorm();
        return lookupMotorSpeedFromDistance.get(distance);
    }

    public static double yawAngleToPos(Pose2d turretPose, Translation2d endPose) {
        return MathUtil.angleModulus(Math.atan2(
                (endPose.getY() - turretPose.getY()), (endPose.getX() - turretPose.getX()))
                - turretPose.getRotation().getRadians());
    }

    public static Pose2d getZone(Pose2d turretPose) {
        double rx = turretPose.getX();
        double ry = turretPose.getY();
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
