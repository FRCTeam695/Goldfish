package frc.robot.subsystems;

import java.util.Optional;
import java.util.Set;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
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
    


    public final double kp_attract = 3.4;

    // we will tune this on the practice field
    public final double kp_repulse = 2;

    public boolean hasDetectedCollision = false;
    public boolean currentlyApplyingRepulsion = false;
    public boolean currentlyFullyAutonomous = false;

    public Trigger isCloseToDestination;
    public Trigger isAtDestination;
    public Trigger collisionDetected;
    public Trigger almostRotatedToSetpoint;
    public Trigger isApplyingRepulsion;
    public Trigger isWithin10cm;
    public Trigger isFullyAutonomous;
    public TrapezoidProfile distanceProfile;

    public Swerve(String[] camNames, TalonFXModule[] modules) {
        super(camNames, modules);

        targetLocationPose = new Pose2d();

        inst = NetworkTableInstance.getDefault();
        sideCarTable = inst.getTable("sidecarTable");  
        scoringLocationSub = sideCarTable.getStringTopic("scoringLocation").subscribe("");
        scoringModeSub = sideCarTable.getStringTopic("currentIntakeMode").subscribe("");

        isCloseToDestination = new Trigger(() -> getDistanceToTranslation(targetLocationPose.getTranslation()) < 2.5);
        isAtDestination = new Trigger(() -> getDistanceToTranslation(targetLocationPose.getTranslation()) < 0.01);
        collisionDetected = new Trigger(()-> hasDetectedCollision);
        almostRotatedToSetpoint = new Trigger(()-> robotRotationError < 20);
        isApplyingRepulsion = new Trigger(()-> currentlyApplyingRepulsion);
        isWithin10cm = new Trigger(() -> getDistanceToTranslation(targetLocationPose.getTranslation()) < 0.1);
        isFullyAutonomous = new Trigger(()-> currentlyFullyAutonomous);
        distanceProfile = new TrapezoidProfile(new TrapezoidProfile.Constraints(Constants.Swerve.MAX_SPEED_METERS_PER_SECONDS_AUTONOMOUS, Constants.Swerve.MAX_ACCELERATION_METERS_PER_SECOND_SQ));
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
    public Command alignToReef(Optional<String> location, DoubleSupplier elevatorTimeToArrival, boolean willRaiseElevator){
        return 
            runOnce(()-> currentlyFullyAutonomous = true)
            .andThen(
            run(()->{
                SmartDashboard.putBoolean("Collision Detected",  collisionDetected.getAsBoolean());
                SmartDashboard.putBoolean("Close to Destination",  isCloseToDestination.getAsBoolean());

                Pose2d robotPose = getSavedPose();
                // if no location is provided, we grab it from networktables
                if(location.isEmpty()){
                    targetLocationPose = getCoralScoringLocation(scoringLocationSub.get());
                }
                // if a location is provided, we just drive to the provided lcoation
                else{
                    targetLocationPose = getCoralScoringLocation(location.get());
                }


                double dx = targetLocationPose.getX()-robotPose.getX();
                double dy = targetLocationPose.getY() - robotPose.getY();

                // calculate attraction forces
                double attractX = kp_attract * dx;
                double attractY = kp_attract * dy;

                double repulsionX = 0;
                double repulsionY = 0;

                boolean elevatorNotInTime = false;

                // calculate the robot relative forward distance we need to go in order to get to the target;
                // if its negative we have to move backwards
                double distanceForward = dx * targetLocationPose.getRotation().getCos() + dy * targetLocationPose.getRotation().getSin();
                double distanceToTarget = getDistanceToTranslation(targetLocationPose.getTranslation());

                // if we are within 20 cm of target its impossible 4 us 2 collide
                boolean willCollideWithReef = distanceForward < 0 && distanceToTarget > 0.2;

                // if we aren't going to collide with the reef then check if we need to apply repulsion vectors by seeing if the elevator will hit any coral/algae already on the reef if we raise rn
                if(!willCollideWithReef){
                    ChassisSpeeds currentRobotChassisSpeeds = getLatestChassisSpeed();
                    double currentSpeedMagnitude = Math.hypot(currentRobotChassisSpeeds.vxMetersPerSecond, currentRobotChassisSpeeds.vyMetersPerSecond);
                    distanceProfile.calculate(0.02, new TrapezoidProfile.State(Math.hypot(dx, dy), currentSpeedMagnitude), new TrapezoidProfile.State(0, 0));
                    double swerveTimeToArrival = distanceProfile.timeLeftUntil(0);

                    SmartDashboard.putNumber("Swerve ETA", swerveTimeToArrival);

                    // if the elevator gets to height BEFORE we arrive at the target position, we aren't going to hit anything
                    elevatorNotInTime = elevatorTimeToArrival.getAsDouble() > swerveTimeToArrival;
                }

                // if we are just testing the auto align and don't plan on actually raising the elevator, 
                // we don't care about elevator hitting anything on the way up
                if(!willRaiseElevator) elevatorNotInTime = false;

                // if we will collide with something
                if( willCollideWithReef || elevatorNotInTime){
                    if(willCollideWithReef) hasDetectedCollision = true;
                    currentlyApplyingRepulsion = true;
                    Transform2d repulsionVector;
                    if(elevatorNotInTime && !willCollideWithReef) {
                        SmartDashboard.putBoolean("strong repulsion", false);
                        repulsionVector = getRepulsionVector(robotPose, 0.6);
                    }
                    // no collision with the reef but the elevator is getting to height late, so we need to back up/slow down
                    else {
                        SmartDashboard.putBoolean("strong repulsion", true);
                        repulsionVector = getRepulsionVector(robotPose, kp_repulse);
                    }
                    repulsionX += repulsionVector.getX();
                    repulsionY += repulsionVector.getY();
                }else{
                    currentlyApplyingRepulsion = false;
                    hasDetectedCollision = false;
                    SmartDashboard.putBoolean("strong repulsion", false);
                }

                SmartDashboard.putNumber("Repulse Speed", Math.hypot(repulsionX, repulsionY));
                SmartDashboard.putNumber("Attract Speed", Math.hypot(attractX, attractY));

                // combine repulsion and attraction forces for the adjusted velocities
                double xSpeed = attractX - repulsionX;
                double ySpeed = attractY - repulsionY;


                //clamp speeds to avoid desaturation killing our rotational movement
                ChassisSpeeds speeds = new ChassisSpeeds(
                    MathUtil.clamp(xSpeed, -Constants.Swerve.MAX_SPEED_METERS_PER_SECONDS_AUTONOMOUS, Constants.Swerve.MAX_SPEED_METERS_PER_SECONDS_AUTONOMOUS), 
                    MathUtil.clamp(ySpeed, -Constants.Swerve.MAX_SPEED_METERS_PER_SECONDS_AUTONOMOUS, Constants.Swerve.MAX_SPEED_METERS_PER_SECONDS_AUTONOMOUS), 
                    getAngularComponentFromRotationOverride(targetLocationPose.getRotation().getDegrees()));
                SmartDashboard.putString("Chassis Speeds Commanded", speeds.toString());
                drive(speeds, true, false);
            }).until(isAtDestination.and(isApplyingRepulsion.negate().and(atRotationSetpoint)))
            .andThen(
                run(()-> {
                    // stops robot when command ends so we don't keep driving bcs of residual speeds
                    drive(new ChassisSpeeds(), false, false);
                })
            )
            ).finallyDo(()->{
                currentlyFullyAutonomous = false;
            });
        // );
    }


    public Command rotateToReefCenter(Supplier<ChassisSpeeds> wantedSpeeds){
        return run(()->{
            Pose2d reefCenter;
            Pose2d robotPose = getSavedPose();
            if(isRedAlliance()){
                reefCenter = Constants.Vision.Red.REEF_CENTER;
            }
            else{
                reefCenter = Constants.Vision.Blue.REEF_CENTER;
            }
            targetLocationPose = reefCenter;
            double dx = reefCenter.getX() - robotPose.getX();
            double dy = reefCenter.getY() - robotPose.getY();
            double theta = Math.toDegrees(Math.atan2(dy, dx));
            SmartDashboard.putNumber("Desired Robot Rotation", theta);
            ChassisSpeeds speeds = wantedSpeeds.get();
            speeds.omegaRadiansPerSecond = getAngularComponentFromRotationOverride(theta);
            drive(speeds, true, true);
        });
    }


    public Command driveForwards(){
        return run(
            ()->{
                driveRobotRelative(new ChassisSpeeds(1, 0, 0), false, false);
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
            drive(speeds, true, false);
        });     
    }


    public Command rotateToDislodgeLocation(Supplier<ChassisSpeeds> commandedSpeeds){
           return  
           runOnce(()->{
            Pose2d robotPose = getSavedPose();
                
            Pose2d[] dislodgePositions = getAlgaeDislodgeLocations();
            double closestDislodge = robotPose.getTranslation().minus(dislodgePositions[0].getTranslation()).getNorm();
            targetLocationPose = new Pose2d(dislodgePositions[0].getX(), dislodgePositions[0].getY(), Rotation2d.fromDegrees(dislodgePositions[0].getRotation().getDegrees() + 180));
            for(int i = 1; i < dislodgePositions.length; ++i){
                double distance = robotPose.getTranslation().minus(dislodgePositions[i].getTranslation()).getNorm();
                if (closestDislodge > distance) {
                    closestDislodge = distance;
                    targetLocationPose = new Pose2d(dislodgePositions[i].getX(), dislodgePositions[i].getY(), Rotation2d.fromDegrees(dislodgePositions[i].getRotation().getDegrees() + 180));
                }
            }
           }).andThen(
            rotateToAngle(()-> targetLocationPose.getRotation().getDegrees(), commandedSpeeds)
            );
    }


    public Command driveBackwardsRobotRelative(){
        return run(()-> {
            driveRobotRelative(new ChassisSpeeds(-1, 0, 0), false, false);
            }).withTimeout(0.5);
    }


    public Command driveToNearestFeed(){
            
        return 
            (run(
            ()->{
                // the current field relative robot pose
                Pose2d robotPose = getSavedPose();

                Translation2d rightFeeder = getFeedLocation("Right").getTranslation();
                Translation2d leftFeeder = getFeedLocation("Left").getTranslation();
                double angle;

                if(leftFeeder.getDistance(robotPose.getTranslation()) < rightFeeder.getDistance(robotPose.getTranslation())){
                    angle = getFeedLocation("Left").getRotation().getDegrees();
                    targetLocationPose = getFeedLocation("Left");
                }
                else{
                    angle = getFeedLocation("Right").getRotation().getDegrees();
                    targetLocationPose = getFeedLocation("Right");
                }

                double dx = targetLocationPose.getX() - robotPose.getX();
                double dy = targetLocationPose.getY() - robotPose.getY();

                SmartDashboard.putNumber("alignment dx", dx);
                SmartDashboard.putNumber("alignment dy", dy);

                double attractX = kp_attract * dx;
                double attractY = kp_attract * dy;

                double repulsionX = 0;
                double repulsionY = 0;

                Transform2d repulsionVector = getRepulsionVector(robotPose, 0.5);
                repulsionX += repulsionVector.getX() < 1 ? 0 : repulsionVector.getX();
                repulsionY += repulsionVector.getY() < 1 ? 0 : repulsionVector.getY();

                SmartDashboard.putNumber("Attract Speed", Math.hypot(attractX, attractY));

                ChassisSpeeds speeds = new ChassisSpeeds(attractX - repulsionX, attractY - repulsionY, getAngularComponentFromRotationOverride(angle));
                SmartDashboard.putString("align to reef speeds", speeds.toString());

                drive(speeds, true, false);
            }
            ).until(() -> getDistanceToTranslation(targetLocationPose.getTranslation()) < 0.05))
            .andThen(runOnce(()-> {
                driveRobotRelative(new ChassisSpeeds(), false, false);
            })).finallyDo(()->{
                currentlyFullyAutonomous = false;
            });
    }


    public Command leftGyroReset(){
        return resetGyro(90);
    }


    public Command rightGyroReset(){
        return resetGyro(-90);
    }


    public void calibrateReefVerticies(){
        reefVerticies[0] = new Pose2d(getReefVertexCalibrationLocation().getX()+Units.inchesToMeters(6.125) + (Constants.Swerve.TRACK_WIDTH_METERS)/2, getReefVertexCalibrationLocation().getY() - (Constants.Swerve.TRACK_WIDTH_METERS)/2 - Units.inchesToMeters(7), new Rotation2d());
        reefVerticies[1] = new Pose2d(getReefVertexCalibrationLocation().getX()+Units.inchesToMeters(6.125) + (Constants.Swerve.TRACK_WIDTH_METERS)/2, getReefVertexCalibrationLocation().getY() + (Constants.Swerve.TRACK_WIDTH_METERS)/2 + Units.inchesToMeters(7), new Rotation2d());
        reefVerticies[2] = new Pose2d(reefVerticies[0].getX()+Units.inchesToMeters(65/2.), reefVerticies[0].getY() - Units.inchesToMeters(20.25), new Rotation2d());
        reefVerticies[3] = new Pose2d(reefVerticies[0].getX()+Units.inchesToMeters(65), reefVerticies[0].getY(), new Rotation2d());
        reefVerticies[4] = new Pose2d(reefVerticies[1].getX()+Units.inchesToMeters(65), reefVerticies[1].getY(), new Rotation2d());
        reefVerticies[5] = new Pose2d(reefVerticies[0].getX()+Units.inchesToMeters(65/2.), reefVerticies[1].getY() + Units.inchesToMeters(20.25), new Rotation2d());
    }


    public Transform2d getRepulsionVector(Pose2d robotPose, double repulsionGain){
        currentlyFullyAutonomous = true;
        double repulsionX = 0;
        double repulsionY = 0;
        for(var vertex : reefVerticies){
            ++inc;
            // get distance to vertex
            Transform2d transformToVertex = vertex.minus(robotPose);
            double vdx = vertex.getX() - robotPose.getX();
            double vdy = vertex.getY() - robotPose.getY();
            double distance = transformToVertex.getTranslation().getNorm();
            
            // magnitude of repulsive force
            double f_mag = repulsionGain/Math.pow(distance,2);

            // unit vector of repulsive force
            double unit_vector_x = vdx/distance;
            double unit_vector_y = vdy/distance;

            // multiply by unit vector to get direction and magnitude
            double f_x = f_mag * unit_vector_x;
            double f_y = f_mag * unit_vector_y;

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


    public Pose2d[] getAlgaeDislodgeLocations(){
        if(isRedAlliance()){
            return Constants.Vision.Red.ALGAE_DISLODGE_POSITIONS;
        }
        else{
            return Constants.Vision.Blue.ALGAE_DISLODGE_POSITIONS;
        }
    }


    public Pose2d getReefVertexCalibrationLocation(){
        Pose2d calibrationLocation;
        if(isRedAlliance()){
            calibrationLocation = Constants.Vision.Red.ALGAE_G_DISLODGE_LOCATION;
        }else{
            calibrationLocation = Constants.Vision.Blue.ALGAE_A_DISLODGE_LOCATION;
        }
        
        return calibrationLocation;
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

    public Command displayVisionConstants(){
        return runOnce(
            ()->{
                calibrateReefVerticies();
                //coral scoring locations
                Set<String> blueCoralKeys = Constants.Vision.Blue.CORAL_SCORING_LOCATIONS.keySet();
                for(String coralKey: blueCoralKeys){
                    String displayString = "Blue Coral " + coralKey;
                    m_field.getObject(displayString).setPose(Constants.Vision.Blue.CORAL_SCORING_LOCATIONS.get(coralKey));
                }
                Set<String> redCoralKeys = Constants.Vision.Red.CORAL_SCORING_LOCATIONS.keySet();
                for(String coralKey: redCoralKeys){
                    String displayString = "Red Coral " + coralKey;
                    m_field.getObject(displayString).setPose(Constants.Vision.Red.CORAL_SCORING_LOCATIONS.get(coralKey));
                }

                //algae scoring locations
                String [] charactersforalgae = {"A" , "C", "E", "G", "I", "K"};
                for(int algaeNum = 0; algaeNum < Constants.Vision.Blue.ALGAE_DISLODGE_POSITIONS.length; algaeNum++){
                    String displayString = "Blue Algae " + charactersforalgae[algaeNum];
                    m_field.getObject(displayString).setPose(Constants.Vision.Blue.ALGAE_DISLODGE_POSITIONS[algaeNum]);
                }
                for(int algaeNum = 0; algaeNum < Constants.Vision.Red.ALGAE_DISLODGE_POSITIONS.length; algaeNum++){
                    String displayString = "Red Algae " + charactersforalgae[algaeNum];
                    m_field.getObject(displayString).setPose(Constants.Vision.Red.ALGAE_DISLODGE_POSITIONS[algaeNum]);
                }

                //feederstaton
                m_field.getObject("Blue Feederstation Left").setPose(Constants.Vision.Blue.FEED_LOCATION_LEFT);
                m_field.getObject("Blue Feederstation Right").setPose(Constants.Vision.Blue.FEED_LOCATION_RIGHT);
                m_field.getObject("Red Feederstation Left").setPose(Constants.Vision.Red.FEED_LOCATION_LEFT);
                m_field.getObject("Red Feederstation Right").setPose(Constants.Vision.Red.FEED_LOCATION_RIGHT);
                //reef Verticies
                for(int vertexNumber = 0; vertexNumber < reefVerticies.length; vertexNumber++){
                    String displayString = "Vertex " + vertexNumber;
                    m_field.getObject(displayString).setPose(reefVerticies[vertexNumber]);
                }
            }
        );
    }



    @Override
    public void periodic(){
        super.periodic();
        m_field.getObject("target location").setPose(targetLocationPose);
        SmartDashboard.putBoolean("Close to Destination", isCloseToDestination.getAsBoolean());
        SmartDashboard.putBoolean("At Destination", isAtDestination.getAsBoolean());
        SmartDashboard.putBoolean("Fully Autonomous", isFullyAutonomous.getAsBoolean());
        SmartDashboard.putNumber("Distance to target", getDistanceToTranslation(targetLocationPose.getTranslation()));
    }
}

