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
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
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
    


    public final double kp_attract = 3.5;

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
    public TrapezoidProfile xProfile;
    public TrapezoidProfile yProfile;
    public TrapezoidProfile distanceProfile;

    public Swerve(String[] camNames, TalonFXModule[] modules, int[] reefTags) {
        super(camNames, modules, reefTags);

        targetLocationPose = new Pose2d();

        inst = NetworkTableInstance.getDefault();
        sideCarTable = inst.getTable("sidecarTable");  
        scoringLocationSub = sideCarTable.getStringTopic("scoringLocation").subscribe("");
        scoringModeSub = sideCarTable.getStringTopic("currentIntakeMode").subscribe("");

        isCloseToDestination = new Trigger(() -> getDistanceToTranslation(targetLocationPose.getTranslation()) < 2.5);
        isAtDestination = new Trigger(() -> getDistanceToTranslation(targetLocationPose.getTranslation()) < 0.02);
        collisionDetected = new Trigger(()-> hasDetectedCollision);
        almostRotatedToSetpoint = new Trigger(()-> robotRotationError < 20);
        isApplyingRepulsion = new Trigger(()-> currentlyApplyingRepulsion);
        isWithin10cm = new Trigger(() -> getDistanceToTranslation(targetLocationPose.getTranslation()) < 0.1);
        isFullyAutonomous = new Trigger(()-> currentlyFullyAutonomous);
        distanceProfile = new TrapezoidProfile(new TrapezoidProfile.Constraints(Constants.Swerve.MAX_TRACKABLE_SPEED_METERS_PER_SECOND, Constants.Swerve.MAX_ACCELERATION_METERS_PER_SECOND_SQ));
        xProfile = new TrapezoidProfile(new TrapezoidProfile.Constraints(Constants.Swerve.MAX_TRACKABLE_SPEED_METERS_PER_SECOND, Constants.Swerve.MAX_ACCELERATION_METERS_PER_SECOND_SQ));
        yProfile = new TrapezoidProfile(new TrapezoidProfile.Constraints(Constants.Swerve.MAX_TRACKABLE_SPEED_METERS_PER_SECOND, Constants.Swerve.MAX_ACCELERATION_METERS_PER_SECOND_SQ));
    }
      

    public Command driveForwards(){
        return run(
            ()->{
                driveRobotRelative(new ChassisSpeeds(1, 0, 0), false, false);
            }
        );
    }

    public Command driveBackwards(){
        return run(()->{
            driveRobotRelative(new ChassisSpeeds(-1, 0, 0), false, false);
        });
    }


    public Command driveBackwardsRobotRelative(){
        return run(()-> {
            driveRobotRelative(new ChassisSpeeds(-1, 0, 0), false, false);
            }).withTimeout(0.5);
    }


    public Command driveToTargetPoseStraight(Pose2d targetPose, double distanceEnd){
            
        return 
            (run(
            ()->{
                // the current field relative robot pose
                Pose2d robotPose = getSavedPose();

                double dx = targetPose.getX() - robotPose.getX();
                double dy = targetPose.getY() - robotPose.getY();

                // convert to unit vector and get direction towards target
                double distance = Math.hypot(dx, dy);
                double unitX = dx / distance;
                double unitY = dy / distance;

                SmartDashboard.putNumber("alignment dx", dx);
                SmartDashboard.putNumber("alignment dy", dy);

                double speed = MathUtil.clamp(kp_attract * distance, -Constants.Swerve.MAX_TRACKABLE_SPEED_METERS_PER_SECOND, Constants.Swerve.MAX_TRACKABLE_SPEED_METERS_PER_SECOND);

                //ChassisSpeeds currentRobotChassisSpeeds = ChassisSpeeds.fromRobotRelativeSpeeds(getLatestChassisSpeed(), robotPose.getRotation());
                // double attractX = xProfile.calculate(0.02, new TrapezoidProfile.State(dx, currentRobotChassisSpeeds.vxMetersPerSecond), new TrapezoidProfile.State(0, 0)).velocity;
                // double attractY = yProfile.calculate(0.02, new TrapezoidProfile.State(dy, currentRobotChassisSpeeds.vyMetersPerSecond), new TrapezoidProfile.State(0, 0)).velocity;
                double attractX;
                double attractY;
                
                attractY = unitY * speed;
                attractX = unitX * speed;
            

                SmartDashboard.putNumber("Attract Speed", Math.hypot(attractX, attractY));
                
                ChassisSpeeds speeds =
                    new ChassisSpeeds(
                        MathUtil.clamp(attractX, -Constants.Swerve.MAX_TRACKABLE_SPEED_METERS_PER_SECOND, Constants.Swerve.MAX_TRACKABLE_SPEED_METERS_PER_SECOND), 
                        MathUtil.clamp(attractY, -Constants.Swerve.MAX_TRACKABLE_SPEED_METERS_PER_SECOND, Constants.Swerve.MAX_TRACKABLE_SPEED_METERS_PER_SECOND),
                    getAngularComponentFromRotationOverride(targetPose.getRotation().getDegrees())
                );
                SmartDashboard.putString("align to reef speeds", speeds.toString());

                drive(speeds, true, false);
            }
            ).until(() -> getDistanceToTranslation(targetPose.getTranslation()) < distanceEnd))
            .andThen(runOnce(()-> {
                this.stopModules();
            })).finallyDo(()->{
                currentlyFullyAutonomous = false;
            });
    }

    public Command driveToIntermediaryPose(Pose2d targetPose, double distanceEnd, double desiredSpeed){
            
        return 
            (run(
            ()->{

                double kP = kp_attract;
                double speed;

                double attractX;
                double attractY;

                // meant to calculate the kP value so the second we hit distanceEnd, the robot speed will be the desired speed.
                kP = desiredSpeed/distanceEnd;
                
                // the current field relative robot pose
                Pose2d robotPose = getSavedPose();

                double dx = targetPose.getX() - robotPose.getX();
                double dy = targetPose.getY() - robotPose.getY();

                // convert to unit vector and get direction towards target
                double distance = Math.hypot(dx, dy);
                double unitX = dx / distance;
                double unitY = dy / distance;

                SmartDashboard.putNumber("alignment dx", dx);
                SmartDashboard.putNumber("alignment dy", dy);
                
                speed = MathUtil.clamp(kP * distance, -Constants.Swerve.MAX_TRACKABLE_SPEED_METERS_PER_SECOND, Constants.Swerve.MAX_TRACKABLE_SPEED_METERS_PER_SECOND);
    
                attractY = unitY * speed;
                attractX = unitX * speed;
            
                SmartDashboard.putNumber("Attract Speed", Math.hypot(attractX, attractY));
                
                ChassisSpeeds speeds =
                    new ChassisSpeeds(
                        MathUtil.clamp(attractX, -Constants.Swerve.MAX_TRACKABLE_SPEED_METERS_PER_SECOND, Constants.Swerve.MAX_TRACKABLE_SPEED_METERS_PER_SECOND), 
                        MathUtil.clamp(attractY, -Constants.Swerve.MAX_TRACKABLE_SPEED_METERS_PER_SECOND, Constants.Swerve.MAX_TRACKABLE_SPEED_METERS_PER_SECOND),
                    getAngularComponentFromRotationOverride(targetPose.getRotation().getDegrees())
                );
                SmartDashboard.putString("align to reef speeds", speeds.toString());

                drive(speeds, true, false);
            }
            ).until(() -> getDistanceToTranslation(targetPose.getTranslation()) < distanceEnd))
            .finallyDo(()->{
                currentlyFullyAutonomous = false;
            });
    }


    public Command driveToTargetPoseWithPath(Pose2d intermediaryPose, double distanceEndFromIntermediary, double speed, Pose2d finalPose, double distanceEndFinal) {
        return driveToIntermediaryPose(intermediaryPose, distanceEndFromIntermediary, speed)
        .andThen(driveToTargetPoseStraight(finalPose, distanceEndFinal));
    }

    public Command driveToTargetPoseCurved(Pose2d targetPose, double distanceEnd){
            
        return 
            (run(
            ()->{
                // the current field relative robot pose
                Pose2d robotPose = getSavedPose();

                double dx = targetPose.getX() - robotPose.getX();
                double dy = targetPose.getY() - robotPose.getY();

                SmartDashboard.putNumber("alignment dx", dx);
                SmartDashboard.putNumber("alignment dy", dy);

                //ChassisSpeeds currentRobotChassisSpeeds = ChassisSpeeds.fromRobotRelativeSpeeds(getLatestChassisSpeed(), robotPose.getRotation());
                // double attractX = xProfile.calculate(0.02, new TrapezoidProfile.State(dx, currentRobotChassisSpeeds.vxMetersPerSecond), new TrapezoidProfile.State(0, 0)).velocity;
                // double attractY = yProfile.calculate(0.02, new TrapezoidProfile.State(dy, currentRobotChassisSpeeds.vyMetersPerSecond), new TrapezoidProfile.State(0, 0)).velocity;
                double attractX;
                double attractY;
                
                attractY = kp_attract * dy;
                attractX = kp_attract * dx;
            

                SmartDashboard.putNumber("Attract Speed", Math.hypot(attractX, attractY));
                
                ChassisSpeeds speeds =
                    new ChassisSpeeds(
                        MathUtil.clamp(attractX, -Constants.Swerve.MAX_TRACKABLE_SPEED_METERS_PER_SECOND, Constants.Swerve.MAX_TRACKABLE_SPEED_METERS_PER_SECOND), 
                        MathUtil.clamp(attractY, -Constants.Swerve.MAX_TRACKABLE_SPEED_METERS_PER_SECOND, Constants.Swerve.MAX_TRACKABLE_SPEED_METERS_PER_SECOND),
                    getAngularComponentFromRotationOverride(targetPose.getRotation().getDegrees())
                );
                SmartDashboard.putString("align to reef speeds", speeds.toString());

                drive(speeds, true, false);
            }
            ).until(() -> getDistanceToTranslation(targetPose.getTranslation()) < distanceEnd))
            .andThen(runOnce(()-> {
                this.stopModules();
            })).finallyDo(()->{
                currentlyFullyAutonomous = false;
            });
    }

    /* public Command driveToTargetPoseStraightTrapezoidal(Pose2d targetPose, double distanceEnd){
                   
        class MotionState {
            double dx, dy, unitX, unitY, distance;
            TrapezoidProfile.State currentState, goalState;
        }
    
        MotionState state = new MotionState();
        
        Timer timer = new Timer();

        return 
            runOnce(
            ()->{
                // the current field relative robot pose

                SmartDashboard.putBoolean("reached destination", false);

                Pose2d robotPose = getSavedPose();

                state.dx = targetPose.getX() - robotPose.getX();
                state.dy = targetPose.getY() - robotPose.getY();

                state.distance = Math.hypot(state.dx, state.dy);

                // convert to unit vector and get direction towards target
                state.unitX = state.dx / state.distance;
                state.unitY = state.dy / state.distance;

                state.currentState = new TrapezoidProfile.State(0, 0); 
                state.goalState = new TrapezoidProfile.State(state.distance - distanceEnd,0);

                timer.reset();
                timer.start();


            }).andThen(run(() -> 
            {
               
                double elapsedTime = timer.get();

                TrapezoidProfile.State desiredState = distanceProfile.calculate(elapsedTime, state.currentState, state.goalState);

                // double speed = MathUtil.clamp(kp_attract * distance, -Constants.Swerve.MAX_TRACKABLE_SPEED_METERS_PER_SECOND, Constants.Swerve.MAX_TRACKABLE_SPEED_METERS_PER_SECOND);

                //ChassisSpeeds currentRobotChassisSpeeds = ChassisSpeeds.fromRobotRelativeSpeeds(getLatestChassisSpeed(), robotPose.getRotation());
                // double attractX = xProfile.calculate(0.02, new TrapezoidProfile.State(dx, currentRobotChassisSpeeds.vxMetersPerSecond), new TrapezoidProfile.State(0, 0)).velocity;
                // double attractY = yProfile.calculate(0.02, new TrapezoidProfile.State(dy, currentRobotChassisSpeeds.vyMetersPerSecond), new TrapezoidProfile.State(0, 0)).velocity;
                double attractX;
                double attractY;

                
                attractY = state.unitY * desiredState.velocity;
                attractX = state.unitX * desiredState.velocity;
            
                SmartDashboard.putNumber("Attract Speed", Math.hypot(attractX, attractY));

                SmartDashboard.putNumber("desiredStateVelocity", desiredState.velocity);
                SmartDashboard.putNumber("Distance to target trapezoid", Math.hypot(targetPose.getX() - getSavedPose().getX(), targetPose.getY() - getSavedPose().getY()) - distanceEnd);

                ChassisSpeeds speeds =
                    new ChassisSpeeds(
                        MathUtil.clamp(attractX, -Constants.Swerve.MAX_TRACKABLE_SPEED_METERS_PER_SECOND, Constants.Swerve.MAX_TRACKABLE_SPEED_METERS_PER_SECOND), 
                        MathUtil.clamp(attractY, -Constants.Swerve.MAX_TRACKABLE_SPEED_METERS_PER_SECOND, Constants.Swerve.MAX_TRACKABLE_SPEED_METERS_PER_SECOND),
                    getAngularComponentFromRotationOverride(targetPose.getRotation().getDegrees())
                );
                SmartDashboard.putString("align to reef speeds", speeds.toString());

                drive(speeds, true, false);
            })
            .until(() -> (Math.abs(getDistanceToTranslation(targetPose.getTranslation()) - distanceEnd)) < 0.05))
            .andThen(runOnce(()-> {
                SmartDashboard.putBoolean("reached destination", true);
                this.stopModules();
            })).finallyDo(()->{
                currentlyFullyAutonomous = false;
            });
    } */

    public Command driveToTargetPoseStraightTrapezoidal(Pose2d targetPose, double distanceEnd){
            
        return 
            (run(
            ()->{

                SmartDashboard.putBoolean("reached destination", false);

                // the current field relative robot pose
                Pose2d robotPose = getSavedPose();

                double dx = targetPose.getX() - robotPose.getX();
                double dy = targetPose.getY() - robotPose.getY();

                // convert to unit vector and get direction towards target
                double distance = Math.hypot(dx, dy);
                double unitX = dx / distance;
                double unitY = dy / distance;

                SmartDashboard.putNumber("alignment dx", dx);
                SmartDashboard.putNumber("alignment dy", dy);
                
                double xvel = ChassisSpeeds.fromRobotRelativeSpeeds(getLatestChassisSpeed(), robotPose.getRotation()).vxMetersPerSecond;
                double yvel = ChassisSpeeds.fromRobotRelativeSpeeds(getLatestChassisSpeed(), robotPose.getRotation()).vyMetersPerSecond;

                double currentVelocity = Math.hypot(xvel, yvel);

                SmartDashboard.putNumber("current field relative velocity", currentVelocity);

                TrapezoidProfile.State goalState = new TrapezoidProfile.State(0, 0); 
                TrapezoidProfile.State currentState = new TrapezoidProfile.State(distance, currentVelocity);
    
                TrapezoidProfile.State desiredState = distanceProfile.calculate(0.02, currentState, goalState);
                
                double attractX;
                double attractY;

                attractY = unitY * -desiredState.velocity;
                attractX = unitX * -desiredState.velocity;
            
                SmartDashboard.putNumber("desired trapezoidal velocity", desiredState.velocity);
                SmartDashboard.putNumber("distance to target trapezoid", distance);

                SmartDashboard.putNumber("attract speed", Math.hypot(attractX, attractY));
                
                ChassisSpeeds speeds =
                    new ChassisSpeeds(
                        MathUtil.clamp(attractX, -Constants.Swerve.MAX_TRACKABLE_SPEED_METERS_PER_SECOND, Constants.Swerve.MAX_TRACKABLE_SPEED_METERS_PER_SECOND), 
                        MathUtil.clamp(attractY, -Constants.Swerve.MAX_TRACKABLE_SPEED_METERS_PER_SECOND, Constants.Swerve.MAX_TRACKABLE_SPEED_METERS_PER_SECOND),
                    getAngularComponentFromRotationOverride(targetPose.getRotation().getDegrees())
                );
                SmartDashboard.putString("align speeds", speeds.toString());

                drive(speeds, true, false);
            }
            ).until(() -> getDistanceToTranslation(targetPose.getTranslation()) < distanceEnd))
            .andThen(runOnce(()-> {
                SmartDashboard.putBoolean("reached destination", true);
                this.stopModules();
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

    @Override
    public void periodic(){
        super.periodic();
        m_field.getObject("target location").setPose(targetLocationPose);
        SmartDashboard.putBoolean("Close to Destination", isCloseToDestination.getAsBoolean());
        SmartDashboard.putBoolean("Is Applying Repulsion", isApplyingRepulsion.getAsBoolean());
        SmartDashboard.putBoolean("At Destination", isAtDestination.getAsBoolean());
        SmartDashboard.putBoolean("Fully Autonomous", isFullyAutonomous.getAsBoolean());
        SmartDashboard.putNumber("Distance to target", getDistanceToTranslation(targetLocationPose.getTranslation()));
    }
}

