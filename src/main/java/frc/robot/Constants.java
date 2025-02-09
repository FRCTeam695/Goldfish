package frc.robot;

import java.util.HashMap;
import java.util.Map;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import frc.BisonLib.BaseProject.Swerve.SwerveConfig;

public class Constants {
    public static final class Swerve {

        public static final SwerveConfig QBConfig = 
                new SwerveConfig(-0.4625, 
                -0.1408, 
                 0.018799, 
                -0.068115, 6.12, 16.5,
                4 * Math.PI, 45, 1, 23.75, 
                23.75, 150.0/7, true, 
                0.006, 1, 
                0, 90, 
                40, 0, 0, 
                0, 0.25, 
                0.11, 0.2, 1);

        public static final SwerveConfig production2025Config =
                new SwerveConfig(
                    //must find all offsets
                -0.45874 + 0.001997 + 0.000244, // front right
                0.4375 - 0.001465 - 0.499512, // front left
                -0.438477 + 0.001465 - 0.499756, // back left
                0.354292 - 0.496094 + 1 - 0.000244, // back right
                 8.14, 12.9, 4 * Math.PI, 
                 // must find this kp
                 65, 1, 
                 // must find wheelbase and track width
                 23.75, 23.75, 150.0/7, true, 
                 // must tune this
                 0.01, 1, 
                 // find this with field
                 0, 
                 // tune stator limit; supply limit doesn't get applied
                 90, 40, 
                 // tune velocity pid and ff
                 0, 0, 0, 
                 0.05, 0.12, 0.2, 1);
        

        public static final Map<String, SwerveConfig> ROBOT_MAP = new HashMap<String, SwerveConfig>() {
            {
                put("QB", QBConfig);
                put("Production_2025", production2025Config);
            }
        };
        

        // CHOOSE WHICH ROBOT YOU'RE USING
        public static final SwerveConfig CHOSEN_CONSTANTS = ROBOT_MAP.get("Production_2025");

        // miscellaneous constants
        public static final double MAX_SPEED_METERS_PER_SECONDS = CHOSEN_CONSTANTS.maxSpeedMetersPerSec;
        public static final double MAX_ANGULAR_SPEED_RAD_PER_SECOND = CHOSEN_CONSTANTS.maxAngularSpeedRadPerSec;
        public static final double TURNING_GEAR_RATIO = CHOSEN_CONSTANTS.turningGearRatio;
        public static final double DRIVING_GEAR_RATIO = CHOSEN_CONSTANTS.drivingGearRatio;
        public static final double WHEEL_CIRCUMFERENCE_METERS = CHOSEN_CONSTANTS.wheelCircumferenceMeters;
        public static final double TURN_WHEEL_KP = CHOSEN_CONSTANTS.turnWheelKP;
        public static final double TURN_WHEEL_KS = CHOSEN_CONSTANTS.turnWheelKS;
        public static final double TURN_WHEEL_KD = CHOSEN_CONSTANTS.turnWheelKD;
        public static final double ROBOT_ROTATION_KP = CHOSEN_CONSTANTS.rotationOverrideKP;
        public static final double PATHPLANNER_OMEGA_KP = CHOSEN_CONSTANTS.pathplannerOmegaKP;
        public static final double PATHPLANNER_TRANSLATION_KP = CHOSEN_CONSTANTS.pathplannerTranslationKP;
        public static final double MAX_WHEEL_ROTATIONAL_SPEED = CHOSEN_CONSTANTS.maxWheelRotationalSpeed;
        public static final double GYRO_DRIFT_COMPENSATION = CHOSEN_CONSTANTS.gyroDriftCompensation;

        public static final double MAX_ACCELERATION_RADIANS_PER_SECOND_SQUARED = CHOSEN_CONSTANTS.maxAngularAccelerationRadPerSecondSquared;
        public static final double DISCRETIZE_TIMESTAMP = 0.02;
        public static final int ODOMETRY_UPDATE_RATE_HZ_INTEGER = 200;
        public static final boolean MODULE_IS_INVERTED = CHOSEN_CONSTANTS.driveMotorInverted;
        public static final double MAX_ACCELERATION_METERS_PER_SECOND_SQ = CHOSEN_CONSTANTS.maxAccelMetersPerSec;
        public static final double SUPPLY_CURRENT_LIMIT = CHOSEN_CONSTANTS.supplyCurrentLimit;
        public static final double STATOR_CURRENT_LIMIT = CHOSEN_CONSTANTS.statorCurrentLimit;

        // configs for drive wheel (closed-loop velocity control)
        public static final double DRIVE_WHEEL_KP = CHOSEN_CONSTANTS.driveWheelKP;
        public static final double DRIVE_WHEEL_KV = CHOSEN_CONSTANTS.driveWheelKV;
        public static final double DRIVE_WHEEL_KS = CHOSEN_CONSTANTS.driveWheelKS;


        public static final int GYRO_ID = 8;
        // front right wheel
        public static final int FRONT_RIGHT_DRIVE_ID = 13;
        public static final int FRONT_RIGHT_TURN_ID = 12;
        public static final int FRONT_RIGHT_CANCODER_ID = 11;
        public static final double FRONT_RIGHT_ABS_ENCODER_OFFSET_ROTATIONS = CHOSEN_CONSTANTS.frontRightOffset;

        // front left wheel
        public static final int FRONT_LEFT_DRIVE_ID = 23;
        public static final int FRONT_LEFT_TURN_ID = 22;
        public static final int FRONT_LEFT_CANCODER_ID = 21;
        public static final double FRONT_LEFT_ABS_ENCODER_OFFSET_ROTATIONS = CHOSEN_CONSTANTS.frontLeftOffset;

        // back left wheel
        public static final int BACK_LEFT_DRIVE_ID = 33;
        public static final int BACK_LEFT_TURN_ID = 32;
        public static final int BACK_LEFT_CANCODER_ID = 31;
        public static final double BACK_LEFT_ABS_ENCODER_OFFSET_ROTATIONS = CHOSEN_CONSTANTS.backLeftOffset;

        // back right wheel
        public static final int BACK_RIGHT_DRIVE_ID = 43;
        public static final int BACK_RIGHT_TURN_ID = 42;
        public static final int BACK_RIGHT_CANCODER_ID = 41;
        public static final double BACK_RIGHT_ABS_ENCODER_OFFSET_ROTATIONS = CHOSEN_CONSTANTS.backRightOffset;

        public static final TrapezoidProfile.Constraints TRAPEZOID_THETA_CONSTRAINTS = new TrapezoidProfile.Constraints(
                MAX_ANGULAR_SPEED_RAD_PER_SECOND, MAX_ACCELERATION_RADIANS_PER_SECOND_SQUARED);

        public static final double TRACK_WIDTH_METERS = CHOSEN_CONSTANTS.trackWidthMeters;
        public static final double WHEEL_BASE_METERS = CHOSEN_CONSTANTS.wheelBaseMeters;

        public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
                CHOSEN_CONSTANTS.frontRightTranslation, // Front right wheel
                CHOSEN_CONSTANTS.frontLeftTranslation, // Front left wheel
                CHOSEN_CONSTANTS.backLeftTranslation, // Back left wheel
                CHOSEN_CONSTANTS.backRightTranslation); // Back right wheel

        public static final Translation2d FRONT_LEFT_TRANSLATION = CHOSEN_CONSTANTS.frontLeftTranslation;
        public static final Translation2d FRONT_RIGHT_TRANSLATION = CHOSEN_CONSTANTS.frontRightTranslation;
        public static final Translation2d BACK_LEFT_TRANSLATION = CHOSEN_CONSTANTS.backLeftTranslation;
        public static final Translation2d BACK_RIGHT_TRANSLATION = CHOSEN_CONSTANTS.backRightTranslation;
    }


    public static final class Vision{

        public static final Transform2d APRIL_TAG_18 = new Transform2d(-5.1164, 0,new Rotation2d(Math.PI));
        

        public static final Pose2d FEED_LOCATION = new Pose2d(0.5, 1.9, new Rotation2d(55));
        public static final Pose2d REEF_A_SCORING_LOCATION = new Pose2d(3.26,4.17, new Rotation2d());
        public static final Pose2d REEF_B_SCORING_LOCATION = new Pose2d(3.26,3.82, new Rotation2d());
        //everything below is random stuff
        public static final Pose2d REEF_C_SCORING_LOCATION = new Pose2d(3,3.82, Rotation2d.fromDegrees(60));
        public static final Pose2d REEF_D_SCORING_LOCATION = new Pose2d(3,3.82, Rotation2d.fromDegrees(60));
        public static final Pose2d REEF_E_SCORING_LOCATION = new Pose2d(3,3.82, Rotation2d.fromDegrees(120));
        public static final Pose2d REEF_F_SCORING_LOCATION = new Pose2d(3,3.82, Rotation2d.fromDegrees(120));
        public static final Pose2d REEF_G_SCORING_LOCATION = new Pose2d(3,3.82, Rotation2d.fromDegrees(180));
        public static final Pose2d REEF_H_SCORING_LOCATION = new Pose2d(3,3.82, Rotation2d.fromDegrees(180));
        public static final Pose2d REEF_I_SCORING_LOCATION = new Pose2d(3,3.82, Rotation2d.fromDegrees(-120));
        public static final Pose2d REEF_J_SCORING_LOCATION = new Pose2d(3,3.82, Rotation2d.fromDegrees(-120));
        public static final Pose2d REEF_K_SCORING_LOCATION = new Pose2d(3,3.82, Rotation2d.fromDegrees(-60));
        public static final Pose2d REEF_L_SCORING_LOCATION = new Pose2d(3,3.82, Rotation2d.fromDegrees(-60));

        public static final HashMap<String, Pose2d> REEF_SCORING_LOCATIONS = new HashMap<String, Pose2d>(){
            {
                put("A", REEF_A_SCORING_LOCATION);
                put("B", REEF_B_SCORING_LOCATION);
                put("C", REEF_C_SCORING_LOCATION);
                put("D", REEF_D_SCORING_LOCATION);
                put("E", REEF_E_SCORING_LOCATION);
                put("F", REEF_F_SCORING_LOCATION);
                put("G", REEF_G_SCORING_LOCATION);
                put("H", REEF_H_SCORING_LOCATION);
                put("I", REEF_I_SCORING_LOCATION);
                put("J", REEF_J_SCORING_LOCATION);
                put("K", REEF_K_SCORING_LOCATION);
                put("L", REEF_L_SCORING_LOCATION);
            }
        };
        
    }
}