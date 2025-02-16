package frc.robot.subsystems;



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
    
    public double lastRecordedDistance;
    public Translation2d startTranslation;
    public boolean hasSeenReefTag;
    public Pose2d scoringLocationPose;
    public Pose2d megatag2Pose;

    public Pose2d[] reefVerticies = new Pose2d[6];

    //NT
    public NetworkTableInstance inst;
    public NetworkTable sideCarTable;
    public StringSubscriber scoringLocationSub; 
    public StringSubscriber scoringModeSub;
    // 2.4, 9       
    public final double kp_attract = 2.6;
    public final double kp_repulse = 1.0;

    public Trigger isCloseToDestination;
    public Trigger isAtDestination;

    public Swerve(String[] camNames, TalonFXModule[] modules) {
        super(camNames, modules);

        hasSeenReefTag = false;
        scoringLocationPose = new Pose2d();
        startTranslation = new Translation2d();
        megatag2Pose = new Pose2d();

        inst = NetworkTableInstance.getDefault();
        sideCarTable = inst.getTable("sidecarTable");  
        scoringLocationSub = sideCarTable.getStringTopic("scoringLocation").subscribe("");
        scoringModeSub = sideCarTable.getStringTopic("currentIntakeMode").subscribe("");

        reefVerticies[0] = new Pose2d(3.174+Units.inchesToMeters(5.5) + (Constants.Swerve.TRACK_WIDTH_METERS)/2, 4.042 - Units.inchesToMeters(7.5), new Rotation2d());
        reefVerticies[1] = new Pose2d(3.174+Units.inchesToMeters(5.5) + (Constants.Swerve.TRACK_WIDTH_METERS)/2, 4.042 + Units.inchesToMeters(7), new Rotation2d());
        reefVerticies[2] = new Pose2d(reefVerticies[0].getX()+Units.inchesToMeters(33.25), reefVerticies[0].getY() - Units.inchesToMeters(8.75), new Rotation2d());
        reefVerticies[3] = new Pose2d(reefVerticies[0].getX()+Units.inchesToMeters(64), reefVerticies[0].getY(), new Rotation2d());
        reefVerticies[4] = new Pose2d(reefVerticies[1].getX()+Units.inchesToMeters(64), reefVerticies[1].getY(), new Rotation2d());
        reefVerticies[5] = new Pose2d(reefVerticies[0].getX()+Units.inchesToMeters(33.25), reefVerticies[1].getY() + Units.inchesToMeters(8.75), new Rotation2d());

        isCloseToDestination = new Trigger(() -> Math.abs(scoringLocationPose.minus(getSavedPose()).getTranslation().getNorm()) < 0.1);
        isAtDestination = new Trigger(() -> Math.abs(scoringLocationPose.minus(getSavedPose()).getTranslation().getNorm()) < 0.005);
    }

    /*
     * The math for all the obstacle avoidance is laid out in this desmos
     * https://www.desmos.com/calculator/z3xqpwls08
     */
    public Command alignToReef(){
        return run(()->{
            Pose2d robotPose = getSavedPose();


            String currentIntakeMode = scoringModeSub.get();
            if(currentIntakeMode.equals("Coral")){
                scoringLocationPose = Constants.Vision.CORAL_SCORING_LOCATIONS.get(scoringLocationSub.get());
            }else if(currentIntakeMode.equals("Algae")) {
                scoringLocationPose = Constants.Vision.ALGAE_SCORING_LOCATIONS.get(scoringLocationSub.get());
            }

            Transform2d transformToScoringLocation = robotPose.minus(scoringLocationPose);

            // calculate attraction forces
            double attractX = kp_attract * transformToScoringLocation.getX();
            double attractY = kp_attract * transformToScoringLocation.getY();

            double repulsionX = 0;
            double repulsionY = 0;

            // calculate the forward distance we need to go in order to get to the target;
            // if its negative we have to move backwards
            double dx = scoringLocationPose.getX()-robotPose.getX();
            double dy = scoringLocationPose.getY() - robotPose.getY();
            SmartDashboard.putString("Scoring Location Pose", scoringLocationPose.toString());

            double distanceForward = dx * scoringLocationPose.getRotation().getCos() + dy * scoringLocationPose.getRotation().getSin();

            SmartDashboard.putBoolean("Collision detected", distanceForward < 0);
            // if we will collide
            if(distanceForward < 0){
                // calculate repulsion vector from all verticies
                for(var vertex : reefVerticies){
                    // get distance to vertex
                    Transform2d transformToVertex = vertex.minus(robotPose);
                    double distance = transformToVertex.getTranslation().getNorm();
                    
                    // magnitude of repulsive force
                    double f_mag = kp_repulse/Math.pow(distance,2);

                    // unit vector of repulsive force
                    double unit_vector_x = transformToVertex.getX()/distance;
                    double unit_vector_y = transformToVertex.getY()/distance;

                    // multiply by unit vector to get direction and magnitude
                    double f_x = f_mag * unit_vector_x;
                    double f_y = f_mag * unit_vector_y;

                    repulsionX += f_x;
                    repulsionY += f_y;
                }
            }
            
            // combine repulsion and attraction forces for the adjusted velocities
            double xSpeed = attractX + repulsionX;
            double ySpeed = attractY + repulsionY;


            ChassisSpeeds speeds = new ChassisSpeeds(-xSpeed, -ySpeed, getAngularComponentFromRotationOverride(edu.wpi.first.math.MathUtil.inputModulus(scoringLocationPose.getRotation().getDegrees(),-180,180)));
            drive(speeds, true);
        }).until(isAtDestination);
    }

    /*
     * this command doesn't run obstacle avoidance
     */
    public Command driveToReefLocation(String location){
        return run(()->{
            Pose2d robotPose = getSavedPose();
            scoringLocationPose = Constants.Vision.CORAL_SCORING_LOCATIONS.get(location);
            Transform2d transformToScoringLocation = robotPose.minus(scoringLocationPose);

            // calculate attraction forces
            double attractX = kp_attract * transformToScoringLocation.getX();
            double attractY = kp_attract * transformToScoringLocation.getY();

            
            ChassisSpeeds speeds = new ChassisSpeeds(-attractX, -attractY, getAngularComponentFromRotationOverride(edu.wpi.first.math.MathUtil.inputModulus(scoringLocationPose.getRotation().getDegrees(),-180,180)));
            drive(speeds, true);
        }).until(isAtDestination);
    }

public Command driveToNearestFeed(){//"A","B","C"...."I"
        
return 
    run(
    ()->{
        // the current field relative robot pose
        Pose2d robotPose = getSavedPose();

        Transform2d transformToFeederRight = robotPose.minus(Constants.Vision.FEED_LOCATION_RIGHT);
        Transform2d transformToFeederLeft = robotPose.minus(Constants.Vision.FEED_LOCATION_LEFT);
        Transform2d closestFeederTransform = transformToFeederLeft.getTranslation().getNorm() < transformToFeederRight.getTranslation().getNorm() ? transformToFeederLeft : transformToFeederRight;


        double attractX = kp_attract * closestFeederTransform.getX();
        double attractY = kp_attract * closestFeederTransform.getY();
        //need to change rotation
        ChassisSpeeds speeds = new ChassisSpeeds(attractX, attractY, getAngularComponentFromRotationOverride(0));
        // ChassisSpeeds speeds = new ChassisSpeeds(1.8 * transformToReef.getX(), 1.8 * transformToReef.getY(), getAngularComponentFromRotationOverride(0));
        SmartDashboard.putString("align to reef speeds", speeds.toString());

        drive(speeds, true);
    }

// runs the command until we are within 1 cm of target
).until(isAtDestination);
}



}

