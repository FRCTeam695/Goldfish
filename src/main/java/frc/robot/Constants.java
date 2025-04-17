package frc.robot;

import java.util.HashMap;
import java.util.Map;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
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

                /*
                 *                 -0.45874, // front right
                0.4375 +0.5, // front left
                -0.438477 +0.5, // back left
                0.354292 +0.5
                 */
        public static final SwerveConfig production2025Config =
                new SwerveConfig(
                    //must find all offsets
                0.0457 + 0.5, // front right
                -0.064209, // front left
                0.061668, // back left
                -0.14446, // back right
                 8.14, Units.metersToFeet(3.6), 4 * Math.PI, 
                 // must find this kp
                 70, 1, 
                 // must find wheelbase and track width
                 23.75, 23.75, 150.0/7, true, 
                 // must tune this
                 0.01, 1, 
                 // find this with field
                 0, 
                 // tune stator limit; supply limit doesn't get applied
                 90, 40, 
                 // tune velocity pid and ff
                 0, 0.145, 0, 
                 0.05, 0.12, 0.2, 1.0/1.003344);
                 //0.99622314806

                 // drive wheel kP = 0.05
                 // drive wheel kV = 0.12
                 // drive wheel kS = 0.2
    
                 // turn wheel kP = 65
                 // turn wheel kS = 50

                 // new turn wheel kp = 70, kd = 0, ks = 0.145

                 //SYSID 
                 // kp = 0.21361
                 // kv = 0.1232
                 // ks = 0.45
                 // all for velocity (drive)

                 //turning motor
                 // kp = 43.8
                 // ks = 0.1111
                 // kv = 2.4877

        public static final Map<String, SwerveConfig> ROBOT_MAP = new HashMap<String, SwerveConfig>() {
            {
                put("QB", QBConfig);
                put("Production_2025", production2025Config);
            }
        };
        

        // CHOOSE WHICH ROBOT YOU'RE USING
        public static final SwerveConfig CHOSEN_CONSTANTS = ROBOT_MAP.get("Production_2025");

        // miscellaneous constants
        public static final double MAX_SPEED_METERS_PER_SECONDS_TELEOP = Units.feetToMeters(12.9);
        public static final double MAX_SPEED_METERS_PER_SECONDS_AUTONOMOUS = 3;
        public static final double MAX_ANGULAR_SPEED_RAD_PER_SECOND = CHOSEN_CONSTANTS.maxAngularSpeedRadPerSec;
        public static final double TURNING_GEAR_RATIO = CHOSEN_CONSTANTS.turningGearRatio;
        public static final double DRIVING_GEAR_RATIO = CHOSEN_CONSTANTS.drivingGearRatio;
        public static final double WHEEL_CIRCUMFERENCE_METERS = CHOSEN_CONSTANTS.wheelCircumferenceMeters;
        public static final double TURN_WHEEL_KP = CHOSEN_CONSTANTS.turnWheelKP;
        public static final double TURN_WHEEL_KS = CHOSEN_CONSTANTS.turnWheelKS;
        public static final double TURN_WHEEL_KD = CHOSEN_CONSTANTS.turnWheelKD;
        public static final double ROBOT_ROTATION_KP = 0.008;
        public static final double PATHPLANNER_OMEGA_KP = CHOSEN_CONSTANTS.pathplannerOmegaKP;
        public static final double PATHPLANNER_TRANSLATION_KP = CHOSEN_CONSTANTS.pathplannerTranslationKP;
        public static final double MAX_WHEEL_ROTATIONAL_SPEED = CHOSEN_CONSTANTS.maxWheelRotationalSpeed;
        public static final double GYRO_DRIFT_COMPENSATION = CHOSEN_CONSTANTS.gyroDriftCompensation;

        public static final double MAX_ACCELERATION_RADIANS_PER_SECOND_SQUARED = CHOSEN_CONSTANTS.maxAngularAccelerationRadPerSecondSquared;
        public static final double DISCRETIZE_TIMESTAMP = 0.02;
        public static final int ODOMETRY_UPDATE_RATE_HZ_INTEGER = 200;
        public static final boolean MODULE_IS_INVERTED = CHOSEN_CONSTANTS.driveMotorInverted;
        public static final double MAX_ACCELERATION_METERS_PER_SECOND_SQ = 7.84;
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

    public static final class Alagizer{
        public static final double dislodgeAngle = 36;
        public static final double safePos = 27;
        public static final double dump = -35.7;
        public static final double holdRamp = -48;

    }


    public static final class Vision{     
        public static final class Blue{
            public static final Pose2d CORAL_A_SCORING_LOCATION = new Pose2d(3.19, 4.20, Rotation2d.fromDegrees(0));
            public static final Pose2d CORAL_B_SCORING_LOCATION = new Pose2d(3.2, 3.85, Rotation2d.fromDegrees(0));
            public static final Pose2d CORAL_C_SCORING_LOCATION = new Pose2d(3.68,3.00, Rotation2d.fromDegrees(60));
            public static final Pose2d CORAL_D_SCORING_LOCATION = new Pose2d(4.00,2.82, Rotation2d.fromDegrees(60));
            public static final Pose2d CORAL_E_SCORING_LOCATION = new Pose2d(4.99,2.82, Rotation2d.fromDegrees(120));
            public static final Pose2d CORAL_F_SCORING_LOCATION = new Pose2d(5.29,3.0, Rotation2d.fromDegrees(120));
            public static final Pose2d CORAL_G_SCORING_LOCATION = new Pose2d(5.78,3.84, Rotation2d.fromDegrees(180));
            public static final Pose2d CORAL_H_SCORING_LOCATION = new Pose2d(5.78,4.2, Rotation2d.fromDegrees(180));
            public static final Pose2d CORAL_I_SCORING_LOCATION = new Pose2d(5.29, 5.06, Rotation2d.fromDegrees(-120));
            public static final Pose2d CORAL_J_SCORING_LOCATION = new Pose2d(4.98,5.23, Rotation2d.fromDegrees(-120));
            public static final Pose2d CORAL_K_SCORING_LOCATION = new Pose2d(4,5.24, Rotation2d.fromDegrees(-60));
            public static final Pose2d CORAL_L_SCORING_LOCATION = new Pose2d(3.7,5.06, Rotation2d.fromDegrees(-60));
            
            
            public static final Pose2d FEED_LOCATION_RIGHT = new Pose2d((1.16 + 1.16)/2, (0.98 + 0.92)/2, Rotation2d.fromDegrees(53.52));
            public static final Pose2d FEED_LOCATION_LEFT = new Pose2d((1.155 + 1.11)/2, (7.075 + 7.11)/2, Rotation2d.fromDegrees(-55.13));

            /*
            public static final Pose2d CORAL_A_SCORING_LOCATION = new Pose2d(3.19, 4.21, Rotation2d.fromDegrees(0));
            public static final Pose2d CORAL_B_SCORING_LOCATION = new Pose2d(3.20, 3.85, Rotation2d.fromDegrees(0));
            public static final Pose2d CORAL_C_SCORING_LOCATION = new Pose2d(3.69,2.99, Rotation2d.fromDegrees(60));
            public static final Pose2d CORAL_D_SCORING_LOCATION = new Pose2d(3.99,2.82, Rotation2d.fromDegrees(60));
            public static final Pose2d CORAL_E_SCORING_LOCATION = new Pose2d(4.98,2.81, Rotation2d.fromDegrees(120));
            public static final Pose2d CORAL_F_SCORING_LOCATION = new Pose2d(5.28,2.99, Rotation2d.fromDegrees(120));
            public static final Pose2d CORAL_G_SCORING_LOCATION = new Pose2d(5.79,3.85, Rotation2d.fromDegrees(180));
            public static final Pose2d CORAL_H_SCORING_LOCATION = new Pose2d(5.78,4.2, Rotation2d.fromDegrees(180));
            public static final Pose2d CORAL_I_SCORING_LOCATION = new Pose2d(5.29,5.06, Rotation2d.fromDegrees(-120));
            public static final Pose2d CORAL_J_SCORING_LOCATION = new Pose2d(4.99,5.23, Rotation2d.fromDegrees(-120));
            public static final Pose2d CORAL_K_SCORING_LOCATION = new Pose2d(4.01,5.24, Rotation2d.fromDegrees(-60));
            public static final Pose2d CORAL_L_SCORING_LOCATION = new Pose2d(3.69,5.06, Rotation2d.fromDegrees(-60));
            
            
            public static final Pose2d FEED_LOCATION_RIGHT = new Pose2d(((1.06+1.23)/2 + 1.06)/2, ((1.03+1.16)/2 + 1.03)/2, Rotation2d.fromDegrees(53.40));
            public static final Pose2d FEED_LOCATION_LEFT = new Pose2d(1.105, (6.99 + 6.88)/2, Rotation2d.fromDegrees(-54.9));
             */


            public static final Pose2d ALGAE_A_DISLODGE_LOCATION = new Pose2d(
                (CORAL_A_SCORING_LOCATION.getX() + CORAL_B_SCORING_LOCATION.getX())/2, 
                (CORAL_A_SCORING_LOCATION.getY() + CORAL_B_SCORING_LOCATION.getY())/2, 
                CORAL_A_SCORING_LOCATION.getRotation());
            public static final Pose2d ALGAE_C_DISLODGE_LOCATION = new Pose2d(
                (CORAL_C_SCORING_LOCATION.getX() + CORAL_D_SCORING_LOCATION.getX())/2, 
                (CORAL_C_SCORING_LOCATION.getY() + CORAL_D_SCORING_LOCATION.getY())/2, 
                CORAL_C_SCORING_LOCATION.getRotation());
            public static final Pose2d ALGAE_E_DISLODGE_LOCATION = new Pose2d(
                (CORAL_E_SCORING_LOCATION.getX() + CORAL_F_SCORING_LOCATION.getX())/2, 
                (CORAL_E_SCORING_LOCATION.getY() + CORAL_F_SCORING_LOCATION.getY())/2, 
                CORAL_E_SCORING_LOCATION.getRotation());
            public static final Pose2d ALGAE_G_DISLODGE_LOCATION = new Pose2d(
                (CORAL_G_SCORING_LOCATION.getX() + CORAL_H_SCORING_LOCATION.getX())/2, 
                (CORAL_G_SCORING_LOCATION.getY() + CORAL_H_SCORING_LOCATION.getY())/2, 
                CORAL_G_SCORING_LOCATION.getRotation());
            public static final Pose2d ALGAE_I_DISLODGE_LOCATION = new Pose2d(
                (CORAL_I_SCORING_LOCATION.getX() + CORAL_J_SCORING_LOCATION.getX())/2, 
                (CORAL_I_SCORING_LOCATION.getY() + CORAL_J_SCORING_LOCATION.getY())/2, 
                CORAL_I_SCORING_LOCATION.getRotation());
            public static final Pose2d ALGAE_K_DISLODGE_LOCATION = new Pose2d(
                (CORAL_K_SCORING_LOCATION.getX() + CORAL_L_SCORING_LOCATION.getX())/2, 
                (CORAL_K_SCORING_LOCATION.getY() + CORAL_L_SCORING_LOCATION.getY())/2, 
                CORAL_K_SCORING_LOCATION.getRotation());

            public static Pose2d REEF_CENTER = new Pose2d(
                (ALGAE_A_DISLODGE_LOCATION.getX() + ALGAE_G_DISLODGE_LOCATION.getX())/2,
                (ALGAE_A_DISLODGE_LOCATION.getY() + ALGAE_G_DISLODGE_LOCATION.getY())/2,
                new Rotation2d()
            );
    
            public static final HashMap<String, Pose2d> CORAL_SCORING_LOCATIONS = new HashMap<String, Pose2d>(){
                {
                    put("A", CORAL_A_SCORING_LOCATION);
                    put("B", CORAL_B_SCORING_LOCATION);
                    put("C", CORAL_C_SCORING_LOCATION);
                    put("D", CORAL_D_SCORING_LOCATION);
                    put("E", CORAL_E_SCORING_LOCATION);
                    put("F", CORAL_F_SCORING_LOCATION);
                    put("G", CORAL_G_SCORING_LOCATION);
                    put("H", CORAL_H_SCORING_LOCATION);
                    put("I", CORAL_I_SCORING_LOCATION);
                    put("J", CORAL_J_SCORING_LOCATION);
                    put("K", CORAL_K_SCORING_LOCATION);
                    put("L", CORAL_L_SCORING_LOCATION);
                }
            };
    
            public static final Pose2d[] ALGAE_DISLODGE_POSITIONS = {
                ALGAE_A_DISLODGE_LOCATION, 
                ALGAE_C_DISLODGE_LOCATION, 
                ALGAE_E_DISLODGE_LOCATION, 
                ALGAE_G_DISLODGE_LOCATION, 
                ALGAE_I_DISLODGE_LOCATION,
                ALGAE_K_DISLODGE_LOCATION
            };
        }   
        /*
         Thu Apr 17 07:43:52.345 2025,E,"new Pose2d(12.58, 5.24, Rotation2d.fromDegrees(-60.08));"
        Thu Apr 17 07:45:12.175 2025,F,"new Pose2d(12.26, 5.06, Rotation2d.fromDegrees(-59.66));"
         */
        public static final class Red{
            public static final Pose2d CORAL_A_SCORING_LOCATION = new Pose2d(14.35, 3.85, Rotation2d.fromDegrees(180));
            public static final Pose2d CORAL_B_SCORING_LOCATION = new Pose2d(14.35, 4.21, Rotation2d.fromDegrees(180));;
            public static final Pose2d CORAL_C_SCORING_LOCATION = new Pose2d(13.86, 5.05, Rotation2d.fromDegrees(-120)); // fudged from 13.87
            public static final Pose2d CORAL_D_SCORING_LOCATION = new Pose2d(13.55, 5.23, Rotation2d.fromDegrees(-120));
            public static final Pose2d CORAL_E_SCORING_LOCATION = new Pose2d(12.58,5.24, Rotation2d.fromDegrees(-60));
            public static final Pose2d CORAL_F_SCORING_LOCATION = new Pose2d(12.26,5.06, Rotation2d.fromDegrees(-60));
            public static final Pose2d CORAL_G_SCORING_LOCATION = new Pose2d(11.76,4.21, Rotation2d.fromDegrees(0.));
            public static final Pose2d CORAL_H_SCORING_LOCATION = new Pose2d(11.77,3.85, Rotation2d.fromDegrees(0.));
            public static final Pose2d CORAL_I_SCORING_LOCATION = new Pose2d(12.25,3.00, Rotation2d.fromDegrees(60.));
            public static final Pose2d CORAL_J_SCORING_LOCATION = new Pose2d(12.56,2.82, Rotation2d.fromDegrees(60.));
            public static final Pose2d CORAL_K_SCORING_LOCATION = new Pose2d(13.54,2.81, Rotation2d.fromDegrees(120));
            public static final Pose2d CORAL_L_SCORING_LOCATION = new Pose2d(13.85,2.99, Rotation2d.fromDegrees(120));
            
            public static final Pose2d FEED_LOCATION_RIGHT = new Pose2d((16.45 + 16.47)/2, (7.02 + 7.05)/2, Rotation2d.fromDegrees(-126.46));
            public static final Pose2d FEED_LOCATION_LEFT = new Pose2d((16.35 + 16.38)/2, (1.03 + 1.00)/2, Rotation2d.fromDegrees((124.4 + 126.46)/2)); // rotation was 124.4, was fudged

            // public static final Pose2d CORAL_A_SCORING_LOCATION = new Pose2d(14.35, (3.83+3.86)/2, Rotation2d.fromDegrees(180));
            // public static final Pose2d CORAL_B_SCORING_LOCATION = new Pose2d(14.35, 4.19, Rotation2d.fromDegrees(180));;
            // public static final Pose2d CORAL_C_SCORING_LOCATION = new Pose2d(13.87, 5.05, Rotation2d.fromDegrees(-120)); // check during practice
            // public static final Pose2d CORAL_D_SCORING_LOCATION = new Pose2d(13.57, 5.23, Rotation2d.fromDegrees(-120)); // check during practice
            // public static final Pose2d CORAL_E_SCORING_LOCATION = new Pose2d(12.58,5.24, Rotation2d.fromDegrees(-60)); // check during practice (bad reading, but matches well with mirrors so we will keep it)
            // public static final Pose2d CORAL_F_SCORING_LOCATION = new Pose2d(12.27,5.06, Rotation2d.fromDegrees(-60)); // check during practice (average between mirror and measured position)
            // public static final Pose2d CORAL_G_SCORING_LOCATION = new Pose2d(11.76,4.19, Rotation2d.fromDegrees(0.)); // check during practice
            // public static final Pose2d CORAL_H_SCORING_LOCATION = new Pose2d(11.76,3.86, Rotation2d.fromDegrees(0.)); // check during practice
            // public static final Pose2d CORAL_I_SCORING_LOCATION = new Pose2d(12.26,2.99, Rotation2d.fromDegrees(60.));
            // public static final Pose2d CORAL_J_SCORING_LOCATION = new Pose2d(12.54,2.83, Rotation2d.fromDegrees(60.));
            // public static final Pose2d CORAL_K_SCORING_LOCATION = new Pose2d(13.54,2.81, Rotation2d.fromDegrees(120));
            // public static final Pose2d CORAL_L_SCORING_LOCATION = new Pose2d(13.85,2.99, Rotation2d.fromDegrees(120));
            
            // public static final Pose2d FEED_LOCATION_RIGHT = new Pose2d((16.43), (7.01), Rotation2d.fromDegrees(-126.6));
            // public static final Pose2d FEED_LOCATION_LEFT = new Pose2d((16.39), (1.02), Rotation2d.fromDegrees(126.48));


            public static final Pose2d ALGAE_A_DISLODGE_LOCATION = new Pose2d(
                (CORAL_A_SCORING_LOCATION.getX() + CORAL_B_SCORING_LOCATION.getX())/2, 
                (CORAL_A_SCORING_LOCATION.getY() + CORAL_B_SCORING_LOCATION.getY())/2, 
                CORAL_A_SCORING_LOCATION.getRotation());
            public static final Pose2d ALGAE_C_DISLODGE_LOCATION = new Pose2d(
                (CORAL_C_SCORING_LOCATION.getX() + CORAL_D_SCORING_LOCATION.getX())/2, 
                (CORAL_C_SCORING_LOCATION.getY() + CORAL_D_SCORING_LOCATION.getY())/2, 
                CORAL_C_SCORING_LOCATION.getRotation());
            public static final Pose2d ALGAE_E_DISLODGE_LOCATION = new Pose2d(
                (CORAL_E_SCORING_LOCATION.getX() + CORAL_F_SCORING_LOCATION.getX())/2, 
                (CORAL_E_SCORING_LOCATION.getY() + CORAL_F_SCORING_LOCATION.getY())/2, 
                CORAL_E_SCORING_LOCATION.getRotation());
            public static final Pose2d ALGAE_G_DISLODGE_LOCATION = new Pose2d(
                (CORAL_G_SCORING_LOCATION.getX() + CORAL_H_SCORING_LOCATION.getX())/2, 
                (CORAL_G_SCORING_LOCATION.getY() + CORAL_H_SCORING_LOCATION.getY())/2, 
                CORAL_G_SCORING_LOCATION.getRotation());
            public static final Pose2d ALGAE_I_DISLODGE_LOCATION = new Pose2d(
                (CORAL_I_SCORING_LOCATION.getX() + CORAL_J_SCORING_LOCATION.getX())/2, 
                (CORAL_I_SCORING_LOCATION.getY() + CORAL_J_SCORING_LOCATION.getY())/2, 
                CORAL_I_SCORING_LOCATION.getRotation());
            public static final Pose2d ALGAE_K_DISLODGE_LOCATION = new Pose2d(
                (CORAL_K_SCORING_LOCATION.getX() + CORAL_L_SCORING_LOCATION.getX())/2, 
                (CORAL_K_SCORING_LOCATION.getY() + CORAL_L_SCORING_LOCATION.getY())/2, 
                CORAL_K_SCORING_LOCATION.getRotation());
            
            public static Pose2d REEF_CENTER = new Pose2d(
                (ALGAE_A_DISLODGE_LOCATION.getX() + ALGAE_G_DISLODGE_LOCATION.getX())/2,
                (ALGAE_A_DISLODGE_LOCATION.getY() + ALGAE_G_DISLODGE_LOCATION.getY())/2,
                new Rotation2d()
            );

            public static final HashMap<String, Pose2d> CORAL_SCORING_LOCATIONS = new HashMap<String, Pose2d>(){
                {
                    put("A", CORAL_A_SCORING_LOCATION);
                    put("B", CORAL_B_SCORING_LOCATION);
                    put("C", CORAL_C_SCORING_LOCATION);
                    put("D", CORAL_D_SCORING_LOCATION);
                    put("E", CORAL_E_SCORING_LOCATION);
                    put("F", CORAL_F_SCORING_LOCATION);
                    put("G", CORAL_G_SCORING_LOCATION);
                    put("H", CORAL_H_SCORING_LOCATION);
                    put("I", CORAL_I_SCORING_LOCATION);
                    put("J", CORAL_J_SCORING_LOCATION);
                    put("K", CORAL_K_SCORING_LOCATION);
                    put("L", CORAL_L_SCORING_LOCATION);
                }
            };

            public static final Pose2d[] ALGAE_DISLODGE_POSITIONS = {
                ALGAE_A_DISLODGE_LOCATION, 
                ALGAE_C_DISLODGE_LOCATION, 
                ALGAE_E_DISLODGE_LOCATION, 
                ALGAE_G_DISLODGE_LOCATION, 
                ALGAE_I_DISLODGE_LOCATION,
                ALGAE_K_DISLODGE_LOCATION
            };
        }
    }
}