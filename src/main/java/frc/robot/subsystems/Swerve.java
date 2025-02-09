package frc.robot.subsystems;


import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.signals.S1CloseStateValue;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringSubscriber;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.BisonLib.BaseProject.Swerve.SwerveBase;
import frc.BisonLib.BaseProject.Swerve.Modules.TalonFXModule;
import frc.robot.Constants;

public class Swerve extends SwerveBase{
    
    public double lastRecordedDistance;
    public Translation2d startTranslation;
    public boolean hasSeenReefTag;
    public Pose2d scoringLocationPose;
    
    //NT
    public NetworkTableInstance inst;
    public NetworkTable sideCarTable;
    public StringSubscriber scoringLocationSub; 

    // 2.4, 9    
    public ProfiledPIDController xController = new ProfiledPIDController(2.4, 0, 0, new TrapezoidProfile.Constraints(Constants.Swerve.MAX_SPEED_METERS_PER_SECONDS, 15));
    public ProfiledPIDController yController = new ProfiledPIDController(2.4, 0, 0, new TrapezoidProfile.Constraints(Constants.Swerve.MAX_SPEED_METERS_PER_SECONDS, 15));

    public Swerve(String[] camNames, TalonFXModule[] modules) {
        super(camNames, modules);

        hasSeenReefTag = false;
        scoringLocationPose = new Pose2d();
        startTranslation = new Translation2d();

        inst = NetworkTableInstance.getDefault();
        sideCarTable = inst.getTable("sideCarTable");  
        scoringLocationSub = sideCarTable.getStringTopic("scoringLocation").subscribe("");
    }
    

    public Command alignToReef(DoubleSupplier height){//"A","B","C"...."I"
        
    return 
        run(
        ()->{

            double xAccel;
            double yAccel;

            xAccel = height.getAsDouble();
            yAccel = height.getAsDouble();

            // the current field relative robot pose
            Pose2d robotPose = getSavedPose();
            scoringLocationPose = Constants.Vision.REEF_SCORING_LOCATIONS.get(scoringLocationSub.get());
            SmartDashboard.putString("NT scoring location", scoringLocationSub.get());
            Transform2d transformToReef = robotPose.minus(scoringLocationPose);

            //xController.setConstraints(new TrapezoidProfile.Constraints(Constants.Swerve.MAX_SPEED_METERS_PER_SECONDS, xAccel));
            //yController.setConstraints(new TrapezoidProfile.Constraints(Constants.Swerve.MAX_SPEED_METERS_PER_SECONDS, yAccel));

            double xSpeed = xController.calculate(transformToReef.getX());
            double ySpeed = yController.calculate(transformToReef.getY());
            ChassisSpeeds speeds = new ChassisSpeeds(xSpeed, ySpeed, getAngularComponentFromRotationOverride(0));
            // ChassisSpeeds speeds = new ChassisSpeeds(1.8 * transformToReef.getX(), 1.8 * transformToReef.getY(), getAngularComponentFromRotationOverride(0));
            SmartDashboard.putString("align to reef speeds", speeds.toString());

            drive(speeds, true);
            }
    
        // runs the command until we are within 1 cm of target
        ).until(() -> Math.abs(scoringLocationPose.minus(getSavedPose()).getTranslation().getNorm()) < 0.01);
    }
    public Command alignToReefLeft(){
        
        return 
            run(
            ()->{
                // the current field relative robot pose
                Pose2d robotPose = getSavedPose();
                scoringLocationPose = Constants.Vision.REEF_A_SCORING_LOCATION;
                Transform2d transformToReef = robotPose.minus(scoringLocationPose);

                double xSpeed = xController.calculate(transformToReef.getX());
                double ySpeed = yController.calculate(transformToReef.getY());
                ChassisSpeeds speeds = new ChassisSpeeds(xSpeed, ySpeed, getAngularComponentFromRotationOverride(0));
                // ChassisSpeeds speeds = new ChassisSpeeds(1.8 * transformToReef.getX(), 1.8 * transformToReef.getY(), getAngularComponentFromRotationOverride(0));
                SmartDashboard.putString("align to reef speeds", speeds.toString());

                drive(speeds, true);


            }
        
        // runs the command until we are within 1 cm of target
        ).until(() -> Math.abs(scoringLocationPose.minus(getSavedPose()).getTranslation().getNorm()) < 0.01);
    }

    public Command alignToReefRight(){//"A","B","C"...."I"
        
    return 
        run(
        ()->{
            // the current field relative robot pose
            Pose2d robotPose = getSavedPose();
            scoringLocationPose = Constants.Vision.REEF_B_SCORING_LOCATION;
           Transform2d transformToReef = robotPose.minus(scoringLocationPose);

            double xSpeed = xController.calculate(transformToReef.getX());
            double ySpeed = yController.calculate(transformToReef.getY());
            ChassisSpeeds speeds = new ChassisSpeeds(xSpeed, ySpeed, getAngularComponentFromRotationOverride(0));
            // ChassisSpeeds speeds = new ChassisSpeeds(1.8 * transformToReef.getX(), 1.8 * transformToReef.getY(), getAngularComponentFromRotationOverride(0));
            SmartDashboard.putString("align to reef speeds", speeds.toString());

            drive(speeds, true);


        }
    
    // runs the command until we are within 1 cm of target
    ).until(() -> Math.abs(scoringLocationPose.minus(getSavedPose()).getTranslation().getNorm()) < 0.01);
}


public Command driveToFeed(){//"A","B","C"...."I"
        
return 
    run(
    ()->{
        // the current field relative robot pose
        Pose2d robotPose = getSavedPose();
        scoringLocationPose = Constants.Vision.FEED_LOCATION;
        Transform2d transformToReef = robotPose.minus(scoringLocationPose);

        double xSpeed = xController.calculate(transformToReef.getX());
        double ySpeed = yController.calculate(transformToReef.getY());
        ChassisSpeeds speeds = new ChassisSpeeds(xSpeed, ySpeed, getAngularComponentFromRotationOverride(0));
        // ChassisSpeeds speeds = new ChassisSpeeds(1.8 * transformToReef.getX(), 1.8 * transformToReef.getY(), getAngularComponentFromRotationOverride(0));
        SmartDashboard.putString("align to reef speeds", speeds.toString());

        drive(speeds, true);


    }

// runs the command until we are within 1 cm of target
).until(() -> Math.abs(scoringLocationPose.minus(getSavedPose()).getTranslation().getNorm()) < 0.01);
}



}

