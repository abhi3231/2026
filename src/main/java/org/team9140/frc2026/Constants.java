package org.team9140.frc2026;

import static edu.wpi.first.units.Units.MetersPerSecond;

import org.team9140.frc2026.generated.TunerConstants;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.signals.RGBWColor;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.util.Units;

public class Constants {
    public static class Ports {
        public static final CANBus CANIVORE = new CANBus("sixseven", "./logs/example.hoot");
        public static final int YAW_MOTOR = 12;
        public static final int PITCH_MOTOR = 0;
        public static final int SHOOTER_MOTOR = 16;
        public static final int SHOOTER_FOLLOWER_MOTOR = 17;
        public static final int INTAKE_SPIN_MOTOR = 24;
        public static final int INTAKE_EXTEND_MOTOR = 9;
        public static final int CLIMBER_MOTOR = 14;
        public static final int HOPPER_SPINNER_MOTOR = 10;
        public static final int HOPPER_FEEDER_MOTOR = 13;

    }

    public static class Cantdle {
        public static final RGBWColor RED = new RGBWColor(255, 0, 0);
        public static final RGBWColor GREEN = new RGBWColor(0, 255, 0);
        public static final RGBWColor BLUE = new RGBWColor(0, 0, 255);
        public static final RGBWColor PINK = new RGBWColor(245, 110, 229);
        public static final RGBWColor ORANGE = new RGBWColor(255, 157, 0);
        public static final RGBWColor PURPLE = new RGBWColor(227, 18, 254);
        public static final RGBWColor OFF = new RGBWColor(0, 0, 0);
        public static final double BLINK_FREQUENCY = 1.0;
    }

    public static class Drive {
        public static double MAX_TELEOP_VELOCITY = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond) * 0.8;
        public static double MAX_TELEOP_ROTATION = Math.toRadians(360);

        public static double MAX_VELOCITY = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond);

        public static double MIN_TELEOP_VELOCITY = 0.05;
        public static double MIN_TELEOP_ROTATION = Math.toRadians(2);
        public static double MIN_AUTON_VELOCITY = 0.05;
        public static double MIN_AUTON_ROTATION = Math.toRadians(2);

        public static final double X_CONTROLLER_P = 2.5 * 3.141592653589793238462643383279502884197169399375;
        public static final double X_CONTROLLER_I = 0.0;
        public static final double X_CONTROLLER_D = 0.015; // TODO: Raise value
        public static final double Y_CONTROLLER_P = X_CONTROLLER_P;
        public static final double Y_CONTROLLER_I = X_CONTROLLER_I;
        public static final double Y_CONTROLLER_D = X_CONTROLLER_D;
        public static final double HEADING_CONTROLLER_P = 8.0; // 8.0
        public static final double HEADING_CONTROLLER_I = 0.0;
        public static final double HEADING_CONTROLLER_D = 0.03; // 0.04

        public static final double REACHEDPOSE_DEBOUNCE = 0.5;
        public static final double BRAKE_IDLE_TIME = 1.0;
        public static final double TELEOP_SHOOTING_VELOCITY_MULTIPLIER = 0.67;

    }

    public static class Shooter {
        public static final double PEAK_FORWARD_TORQUE = 40.0;
        public static final double IDLE_VOLTAGE = 6.7;
        public static final double SPEED_AT_IDLE = 0.0;

        public static final double SHOOTER_KS = 0;
        public static final double SHOOTER_KV = 0;
        public static final double SHOOTER_KA = 0;
        public static final double SHOOTER_KP = 100;
        public static final double SHOOTER_KI = 0;
        public static final double SHOOTER_KD = 0;

        public static final double FLYWHEEL_GEAR_RATIO = 26.0 / 24.0;

        public static final double AUTO_IDLE_TIMESTAMP = 2;
    }

    public static class Climber {
        public static final double GEAR_RATIO = 25;

        public static final double STATOR_CURRENT_LIMIT = 80.0;
        public static final double SUPPLY_CURRENT_LIMIT = 40.0;

        public static final double FORWARD_SOFT_LIMIT_THRESHOLD = 0;
        public static final double REVERSE_SOFT_LIMIT_THRESHOLD = 0;

        public static final double EXTENSION_VOLTAGE = 12.0;
        public static final double EXTEND_POSITION = 1.0;

        public static final double SIM_PERIOD = 0.004;
        public static final double MIN_HEIGHT = Units.inchesToMeters(0);
        public static final double MAX_HEIGHT = Units.inchesToMeters(8);

        public static final double SPOOL_DIAMETER = Units.inchesToMeters(15);
        public static final double SPOOL_CIRCUMFERENCE = Units.inchesToMeters(SPOOL_DIAMETER * Math.PI);

        public static final double MECHANISM_WIDTH = 3;
        public static final double MECHANISM_HEIGHT = 3;
    }

    public static class Intake {
        public static final double EXTENSION_GEAR_RATIO = 36.0 / 15.0 * 50.0 / 11.0;
        public static final double TOLERANCE = Units.inchesToMeters(0.5);

        public static final int PINION_TEETH = 10;
        public static final double PINION_DP = 10;
        public static final double PINION_CIRCUMFERENCE = Units.inchesToMeters(PINION_TEETH / PINION_DP * Math.PI);
        public static final double INTAKE_VOLTAGE = 12.0;
        public static final double INTAKE_OFF = 0.0;

        public static final double ROLLER_STATOR_CURRENT_LIMIT = 80.0;
        public static final double EXTEND_STATOR_CURRENT_LIMIT = 15.0;

        public static final double ROLLER_SUPPLY_CURRENT_LIMIT = 30.0;
        public static final double EXTEND_SUPPLY_CURRENT_LIMIT = 60.0;

        public static final double MOTION_MAGIC_CRUISE_VELOCITY = 64;
        public static final double MOTION_MAGIC_ACCELERATION = 64;
        public static final double EXTEND_KP = 200;

        public static final double ARM_IN_POSITION = 0.0;
        public static final double ARM_OUT_POSITION = 0.29;

        public static final double FORWARD_SOFT_LIMIT_THRESHOLD = ARM_OUT_POSITION / PINION_CIRCUMFERENCE;
        public static final double REVERSE_SOFT_LIMIT_THRESHOLD = 0;

        public static final double MIN_HEIGHT = ARM_IN_POSITION;
        public static final double MAX_HEIGHT = ARM_OUT_POSITION;

        public static final double LIGAMENT_LENGTH = 12;
        public static final double MECHANISM_LENGTH = 2;
        public static final double MECHANISM_HEIGHT = 2;

        public static final double SIM_PERIOD = 0.004;

        public static final double TURN_OFF_TIME = 3;
    }

    public static class Hopper {
        public static final double SPINNER_STATOR_CURRENT_LIMIT = 80;
        public static final double FEEDER_STATOR_CURRENT_LIMIT = 80;
        public static final double SPINNER_SUPPLY_CURRENT_LIMIT = 40;
        public static final double FEEDER_SUPPLY_CURRENT_LIMIT = 40;

        public static final double SPINNER_VOLTAGE = 12;
        public static final double FEEDER_VOLTAGE = 12;

        public static final double REVERSE_FEEDER_TIME = 1.0;
    }

    public static class Turret {
        public static final double STATOR_CURRENT_LIMIT = 60.0;
        public static final double SUPPLY_CURRENT_LIMIT = 40.0;

        public static final double MM_ACCELERATION = 12;
        public static final double MM_CRUISE_VELOCITY = 8;

        public static final double KS = 0;
        public static final double KV = 0;
        public static final double KA = 0;
        public static final double KP = 1000;
        public static final double KI = 0;
        public static final double KD = 50;

        public static final double ADJUST_VOLTAGE = 2.0;
        public static final double FORWARD_SOFT_LIMIT_THRESHOLD = 0.3; // rot
        public static final double REVERSE_SOFT_LIMIT_THRESHOLD = -0.3; // rot
        public static final double GEAR_RATIO = 90.0 / 10.0 * 50.0 / 11.0;

        public static final double OVERTURN_LOOKAHEAD_TIME = 1.5;
        public static final Transform2d POSITION_TO_ROBOT = new Transform2d(-0.178, -0.114, new Rotation2d(-Math.PI));

        public static final double TURN_SHOOTER_OFF_TIME = 1.0;
    }

    public static class Vision {
        public static final String[] CAMERA_NAMES = {"limelight-a", "limelight-b"};
        public static Transform3d[] ROBOT_TO_CAM = {null, null};
    }
}