// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.BisonLib.BaseProject.Controller.EnhancedCommandController;

// import frc.robot.Subsystems.CoralGripper2Motors;
import frc.robot.subsystems.Swerve;
import frc.BisonLib.BaseProject.Swerve.Modules.TalonFXModule;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import edu.wpi.first.wpilibj2.command.PrintCommand;

import static edu.wpi.first.wpilibj2.command.Commands.*;

import java.util.Optional;

import edu.wpi.first.networktables.IntegerSubscriber;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.BisonLib.BaseProject.Controller.EnhancedCommandController;
import frc.BisonLib.BaseProject.Swerve.Modules.TalonFXModule;
// import frc.robot.Subsystems.CoralGripper2Motors;
import frc.robot.subsystems.Swerve;


/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {

  public final Swerve Swerve;
  public IntegerSubscriber scoringHeight;
  
  
  
  SendableChooser<Command> autoChooser = new SendableChooser<>();

  public int[] reefTags = {6,7,8,9,10,11,17,18,19,20,21,22};


  private final TalonFXModule[] modules = new TalonFXModule[] 
          {
            new TalonFXModule(Constants.Swerve.FRONT_RIGHT_DRIVE_ID, Constants.Swerve.FRONT_RIGHT_TURN_ID, Constants.Swerve.FRONT_RIGHT_ABS_ENCODER_OFFSET_ROTATIONS, Constants.Swerve.FRONT_RIGHT_CANCODER_ID, 0),
            new TalonFXModule(Constants.Swerve.FRONT_LEFT_DRIVE_ID, Constants.Swerve.FRONT_LEFT_TURN_ID, Constants.Swerve.FRONT_LEFT_ABS_ENCODER_OFFSET_ROTATIONS, Constants.Swerve.FRONT_LEFT_CANCODER_ID, 1),
            new TalonFXModule(Constants.Swerve.BACK_LEFT_DRIVE_ID, Constants.Swerve.BACK_LEFT_TURN_ID, Constants.Swerve.BACK_LEFT_ABS_ENCODER_OFFSET_ROTATIONS, Constants.Swerve.BACK_LEFT_CANCODER_ID, 2),
            new TalonFXModule(Constants.Swerve.BACK_RIGHT_DRIVE_ID, Constants.Swerve.BACK_RIGHT_TURN_ID, Constants.Swerve.BACK_RIGHT_ABS_ENCODER_OFFSET_ROTATIONS, Constants.Swerve.BACK_RIGHT_CANCODER_ID, 3)
          };

  private final String[] camNames = {"limelight-right"};
  public static final EnhancedCommandController driver =
      new EnhancedCommandController(0);

    

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    Swerve = new Swerve(camNames, modules, reefTags);


    
    scoringHeight = NetworkTableInstance.getDefault().getTable("sidecarTable").getIntegerTopic("scoringLevel").subscribe(1);

    // SmartDashboarding subsystems allow you to see what commands they are running
    SmartDashboard.putData("Swerve Subsystem", Swerve);

    // Configure the trigger bindings
    configureBindings();
    configureDefaultCommands();


    
    

    DataLogManager.start();
  }


  public Runnable getOdometryUpdater(){
    return Swerve::updateOdometryWithKinematics;
  }
  
  
  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {

   


    // indication for human player to drop coral
    
    

    // make sure you gyro reset by aligning with the reef, not eyeballing it
    driver.back().and(driver.start()).onTrue(Swerve.resetGyro());

    // left gyro reset before auton
    driver.povLeft().and(new Trigger(()-> DriverStation.isDisabled())).onTrue(
        Swerve.leftGyroReset()
    );

    
    // right gyro reset before auton
    driver.povRight().onTrue(
      new ConditionalCommand(
        Swerve.rightGyroReset(), 
        //Elevator.goToScoringHeight().until(Elevator.atSetpoint).andThen(Coralizer.ejectCoral()).andThen(Coralizer.runIntakeAndCoralizer(()-> 0).withTimeout(0.01)).andThen(new WaitCommand(5)),
        new WaitCommand(0), 
        ()-> DriverStation.isDisabled()
      )
    );

  

    // driver.b().onFalse(
    //   Alagizer.goToPosition(()-> Constants.Alagizer.dump).until(Alagizer.atSetpoint)
    //   .andThen(
    //     new WaitCommand(0.25)
    //   )
    //   .andThen(Alagizer.dump())
    // );


   
    driver.b().whileTrue( 
      (Swerve.driveForwards().until(driver.flick).andThen(Swerve.driveBackwards().until(driver.flick))).repeatedly()
    
    );
    


    // display all calibrated field constants on glass
    driver.leftStick().onTrue(Swerve.displayVisionConstants().ignoringDisable(true));

    // driver.rightStick().whileTrue(
    //   Elevator.setHeightLevel(Heights.L2).until(Elevator.atSetpoint).andThen(
    //     parallel(
    //       Coralizer.fastEjectCoral(),
    //       Elevator.slowRaise(-0.1)
    //     ).withTimeout(0.4)
    //   ).andThen(new WaitCommand(0.6))
    //   .andThen(Coralizer.runCoralizer(()-> 0).alongWith(Elevator.slowRaise(0)))
    // );

    
  }

  public void configureDefaultCommands(){
    // This is the Swerve subsystem default command, this allows the driver to drive the robot
    Swerve.setDefaultCommand
      (
        run
          (
            ()-> 
              Swerve.teleopDefaultCommand(
                driver::getRequestedChassisSpeeds, // ()-> driver.getRequestedChassisSpeeds()
                true
              )
              ,
              Swerve
          ).withName("Swerve Drive Command")
      );


      //Gripper.setDefaultCommand(Gripper.stop());
  }

  public Command logTrickshotTrue(){
    return runOnce(()-> {SmartDashboard.putBoolean("Trickshot", true);});
  }


  // The command specified in here is run in autonomous
  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }


  


  private Command updateTelemetryState(int state) {
    return runOnce(()-> SmartDashboard.putNumber("Align State", state));
  }
}