package org.team9140.lib;

import java.util.Optional;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;

public class Util {
    private final static double EPSILON = 0.00000001;

    private static DriverStation.Alliance alliance = null;

    public static final Rotation2d NOROTATION = new Rotation2d();

    public static void updateAlliance() {
        alliance = DriverStation.getAlliance().orElse(null);
    }

    public static Optional<DriverStation.Alliance> getAlliance() {
        if (alliance == null) updateAlliance();
        return Optional.ofNullable(alliance);
    }

    public static boolean epsilonEquals(double a, double b, double epsilon) {
        return Math.abs(a - b) < epsilon;
    }

    public static boolean epsilonEquals(double a, double b) {
        return epsilonEquals(a, b, EPSILON);
    }

    public static final double TRANSLATION_E = Units.inchesToMeters(1.5);
    public static final double ROTATION_E = Units.degreesToRadians(3);

    public static boolean epsilonEquals(Pose2d a, Pose2d b) {
        boolean transValid = a.getTranslation().getDistance(b.getTranslation()) < TRANSLATION_E;
        boolean rotValid = rotationEpsilonEquals(
                a.getRotation(), b.getRotation(), ROTATION_E);

        return transValid && rotValid;
    }

    public static boolean rotationEpsilonEquals(Rotation2d a, Rotation2d b, double epsilon) {
        return Math.abs(MathUtil.angleModulus(a.getRadians() - b.getRadians())) <= epsilon;
    }

    public static boolean rotationEpsilonEquals(Rotation2d a, Rotation2d b) {
        return rotationEpsilonEquals(a, b, Math.toRadians(5.0));
    }

    public static double clamp(double val, double limit) {
        if (val > limit) {
            return limit;
        } else return Math.max(val, -limit);
    }

    private final static double defaultDeadband = 0.12;

    public static double applyDeadband(double in, double deadband) {
        if (Math.abs(in) < deadband) {
            return 0.0;
        } else if (in > 0) {
            return (in - deadband) / (1.0 - deadband);
        } else {
            return (in + deadband) / (1.0 - deadband);
        }
    }

    public static double applyDeadband(double in) {
        return applyDeadband(in, defaultDeadband);
    }
}
