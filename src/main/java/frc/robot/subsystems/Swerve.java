package frc.robot.subsystems;

import java.util.Optional;
import java.util.function.Supplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringSubscriber;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.BisonLib.BaseProject.Swerve.SwerveBase;
import frc.BisonLib.BaseProject.Swerve.Modules.TalonFXModule;
import frc.robot.Constants;


public class Swerve extends SwerveBase{
    
    public Pose2d targetLocationPose;

    public Pose2d[] reefVerticies = new Pose2d[6];

    //NT
    public NetworkTableInstance inst;
    public NetworkTable sideCarTable;
    public StringSubscriber scoringLocationSub; 
    public StringSubscriber scoringModeSub;
    

    // probably turn this up, its low rn (for reference QB's attraction kp was 2.7, 
    // when ur tuning make sure ur just running the autoalign without the elevator)
    public final double kp_attract = 2.7;

    // we will tune this on the practice field
    public final double kp_repulse = 1;

    public boolean hasDetectedCollision = false;

    public Trigger isCloseToDestination;
    public Trigger isAtDestination;
    public Trigger collisionDetected;

    public Swerve(String[] camNames, TalonFXModule[] modules) {
        super(camNames, modules);

        targetLocationPose = new Pose2d();

        inst = NetworkTableInstance.getDefault();
        sideCarTable = inst.getTable("sidecarTable");  
        scoringLocationSub = sideCarTable.getStringTopic("scoringLocation").subscribe("");
        scoringModeSub = sideCarTable.getStringTopic("currentIntakeMode").subscribe("");

        reefVerticies[0] = new Pose2d(getReefVertexCalibrationLocation().getX()+Units.inchesToMeters(6.125) + (Constants.Swerve.TRACK_WIDTH_METERS)/2, getReefVertexCalibrationLocation().getY() - (Constants.Swerve.TRACK_WIDTH_METERS)/2 - Units.inchesToMeters(7), new Rotation2d());
        reefVerticies[1] = new Pose2d(getReefVertexCalibrationLocation().getX()+Units.inchesToMeters(5.5) + (Constants.Swerve.TRACK_WIDTH_METERS)/2, getReefVertexCalibrationLocation().getY() + (Constants.Swerve.TRACK_WIDTH_METERS)/2 + Units.inchesToMeters(7), new Rotation2d());
        reefVerticies[2] = new Pose2d(reefVerticies[0].getX()+Units.inchesToMeters(65/2.), reefVerticies[0].getY() - Units.inchesToMeters(20.25), new Rotation2d());
        reefVerticies[3] = new Pose2d(reefVerticies[0].getX()+Units.inchesToMeters(65), reefVerticies[0].getY(), new Rotation2d());
        reefVerticies[4] = new Pose2d(reefVerticies[1].getX()+Units.inchesToMeters(65), reefVerticies[1].getY(), new Rotation2d());
        reefVerticies[5] = new Pose2d(reefVerticies[0].getX()+Units.inchesToMeters(65/2.), reefVerticies[1].getY() + Units.inchesToMeters(20.25), new Rotation2d());

        isCloseToDestination = new Trigger(() -> Math.abs(targetLocationPose.getTranslation().minus(getSavedPose().getTranslation()).getNorm()) < 0.4);
        isAtDestination = new Trigger(() -> Math.abs(targetLocationPose.getTranslation().minus(getSavedPose().getTranslation()).getNorm()) < 0.01);
        collisionDetected = new Trigger(()-> hasDetectedCollision);
    }
    

    /**
     * The math for all the obstacle avoidance is laid out in this desmos
     * https://www.desmos.com/calculator/z3xqpwls08
     * 
     * @param location If you want the robot to auto align to a specific location, 
     *                 you can supply the location in an optional (we use this for auton).
     *                 If you supply an empty optional, then it pulls the location to 
     *                 align to off networktables from the operator interface
     */
    public Command alignToReef(Optional<String> location){
        return 
        runOnce(()-> hasDetectedCollision = true)
        .andThen(
            run(()->{
                SmartDashboard.putBoolean("Collision Detected",  collisionDetected.getAsBoolean());
                SmartDashboard.putBoolean("Close to Destination",  isCloseToDestination.getAsBoolean());

                Pose2d robotPose = getSavedPose();
                // if no location is provided, we grab it from networktables
                if(location.isEmpty()){
                    String currentIntakeMode = scoringModeSub.get();
                    if(currentIntakeMode.equals("Coral")){
                        targetLocationPose = getCoralScoringLocation(scoringLocationSub.get());
                    }else{
                        targetLocationPose = robotPose;
                    }
                }
                // if a location is provided, we just drive to the provided lcoation
                else{
                    targetLocationPose = getCoralScoringLocation(scoringLocationSub.get());
                }


                double dx = targetLocationPose.getX()-robotPose.getX();
                double dy = targetLocationPose.getY() - robotPose.getY();

                // calculate attraction forces
                double attractX = kp_attract * dx;
                double attractY = kp_attract * dy;

                double repulsionX = 0;
                double repulsionY = 0;

                // calculate the forward distance we need to go in order to get to the target;
                // if its negative we have to move backwards
                double distanceForward = dx * targetLocationPose.getRotation().getCos() + dy * targetLocationPose.getRotation().getSin();

                // if we will collide
                if(distanceForward < 0){
                    hasDetectedCollision = true;
                    Transform2d repulsionVector = getRepulsionVector(robotPose);
                    repulsionX += repulsionVector.getX();
                    repulsionY += repulsionVector.getY();
                }else{
                    hasDetectedCollision = false;
                }

                // combine repulsion and attraction forces for the adjusted velocities
                double xSpeed = attractX - repulsionX;
                double ySpeed = attractY - repulsionY;

                //clamp speeds to avoid desaturation killing our rotational movement
                ChassisSpeeds speeds = new ChassisSpeeds(
                    MathUtil.clamp(xSpeed, -Constants.Swerve.MAX_SPEED_METERS_PER_SECONDS, Constants.Swerve.MAX_SPEED_METERS_PER_SECONDS), 
                    MathUtil.clamp(ySpeed, -Constants.Swerve.MAX_SPEED_METERS_PER_SECONDS, Constants.Swerve.MAX_SPEED_METERS_PER_SECONDS), 
                    getAngularComponentFromRotationOverride(targetLocationPose.getRotation().getDegrees()));
                SmartDashboard.putString("Chassis Speeds Commanded", speeds.toString());
                drive(speeds, true);
            }).until(isAtDestination)
        );
    }

    public Command driveForward(){
        return run(
            ()->{
                drive(new ChassisSpeeds(1, 0, 0), true);
            }
        );
    }

    public Command rotateToNearestFeed(Supplier<ChassisSpeeds> wantedVels){
        return
        run(()->{
            // the current field relative robot pose
            Pose2d robotPose = getSavedPose();

            Translation2d transformToFeederRight = robotPose.getTranslation().minus(getFeedLocation("Right").getTranslation());
            Translation2d transformToFeederLeft = robotPose.getTranslation().minus(getFeedLocation("Left").getTranslation());
            double angle;

            // closer to left station
            if(transformToFeederLeft.getNorm() < transformToFeederRight.getNorm()){
                angle = getFeedLocation("Left").getRotation().getDegrees();
            }
            // closer to right station
            else{
                angle = getFeedLocation("Right").getRotation().getDegrees();
            }

            ChassisSpeeds speeds = wantedVels.get();
            speeds.omegaRadiansPerSecond = getAngularComponentFromRotationOverride(angle);
            drive(speeds, true);
        });     
    }


    public Command driveToNearestFeed(){//"A","B","C"...."I"
            
        return 
            run(
            ()->{
                // the current field relative robot pose
                Pose2d robotPose = getSavedPose();

                Translation2d transformToFeederRight = robotPose.getTranslation().minus(getFeedLocation("Right").getTranslation());
                Translation2d transformToFeederLeft = robotPose.getTranslation().minus(getFeedLocation("Left").getTranslation());
                Translation2d closestFeederTransform;
                double angle;
                if(transformToFeederLeft.getNorm() < transformToFeederRight.getNorm()){
                    closestFeederTransform = transformToFeederLeft;
                    angle = getFeedLocation("Left").getRotation().getDegrees();
                    targetLocationPose = getFeedLocation("Left");
                }
                else{
                    closestFeederTransform = transformToFeederRight;
                    angle = getFeedLocation("Right").getRotation().getDegrees();
                    targetLocationPose = getFeedLocation("Right");
                }

                double attractX = kp_attract * closestFeederTransform.getX();
                double attractY = kp_attract * closestFeederTransform.getY();

                double repulsionX = 0;
                double repulsionY = 0;

                Transform2d repulsionVector = getRepulsionVector(robotPose);
                repulsionX += repulsionVector.getX();
                repulsionY += repulsionVector.getY();

                //need to change rotation
                ChassisSpeeds speeds = new ChassisSpeeds(-attractX - repulsionX, -attractY - repulsionY, getAngularComponentFromRotationOverride(angle));
                // ChassisSpeeds speeds = new ChassisSpeeds(1.8 * transformToReef.getX(), 1.8 * transformToReef.getY(), getAngularComponentFromRotationOverride(0));
                SmartDashboard.putString("align to reef speeds", speeds.toString());

                drive(speeds, true);
            }
            ).until(() -> Math.abs(targetLocationPose.getTranslation().minus(getSavedPose().getTranslation()).getNorm()) < 0.05);
    }

    public Transform2d getRepulsionVector(Pose2d robotPose){
        double repulsionX = 0;
        double repulsionY = 0;
        int inc = 0;
        for(var vertex : reefVerticies){
            ++inc;
            m_field.getObject(""+inc).setPose(vertex);
            // get distance to vertex
            Transform2d transformToVertex = vertex.minus(robotPose);
            double vdx = vertex.getX() - robotPose.getX();
            double vdy = vertex.getY() - robotPose.getY();
            double distance = transformToVertex.getTranslation().getNorm();
            
            // magnitude of repulsive force
            double f_mag = kp_repulse/Math.pow(distance,2);

            // unit vector of repulsive force
            double unit_vector_x = vdx/distance;
            double unit_vector_y = vdy/distance;

            // multiply by unit vector to get direction and magnitude
            double f_x = f_mag * unit_vector_x;
            double f_y = f_mag * unit_vector_y;

            // we don't want the repulsion to interfere with our PID align when far away from reef
            f_x = Math.abs(f_x) < 0.5 ? 0 : f_x;
            f_y = Math.abs(f_y) < 0.5 ? 0 : f_y;

            repulsionX += f_x;
            repulsionY += f_y;
        }

        return new Transform2d(repulsionX, repulsionY, new Rotation2d());
    }


    /*
     * returns coral scoring location on each alliance
     * Location A-L
     */
    public Pose2d getCoralScoringLocation(String location){
        Pose2d score_location;
        if(isRedAlliance()){
            score_location = Constants.Vision.Red.CORAL_SCORING_LOCATIONS.get(location);
        }else{
            score_location = Constants.Vision.Blue.CORAL_SCORING_LOCATIONS.get(location);
        }
        
        return score_location;
    }


    public Pose2d getReefVertexCalibrationLocation(){
        Pose2d score_location;
        if(isRedAlliance()){
            score_location = Constants.Vision.Red.VERTEX_CALIBRATION_POSITION;
        }else{
            score_location = Constants.Vision.Blue.VERTEX_CALIBRATION_POSITION;
        }
        
        return score_location;
    }


    /*
     * returns the pose of feed location
     * either L or R
     * (left) or (right)
     * 
     * if given invalid location will return right value
     */
    public Pose2d getFeedLocation(String location){
        Pose2d feed_location = new Pose2d();
        if(isRedAlliance()){
            
            if(location.equals("Left")){
                feed_location = Constants.Vision.Red.FEED_LOCATION_LEFT;
            } else{//right
                feed_location = Constants.Vision.Red.FEED_LOCATION_RIGHT;
            }   
            
        }else{
            if(location.equals("Left")){
                feed_location = Constants.Vision.Blue.FEED_LOCATION_LEFT;
            } else{//right
                feed_location = Constants.Vision.Blue.FEED_LOCATION_RIGHT;
            }   
        }
        
        return feed_location;
    }

    
    @Override
    public void periodic(){
        super.periodic();
        m_field.getObject("target location").setPose(targetLocationPose);
    }
}

