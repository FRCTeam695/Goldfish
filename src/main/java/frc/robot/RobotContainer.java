// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.BisonLib.BaseProject.Controller.EnhancedCommandController;

// import frc.robot.Subsystems.CoralGripper2Motors;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.Coralizer;
import frc.BisonLib.BaseProject.Swerve.Modules.TalonFXModule;

import frc.robot.subsystems.DuoTalonLift;
import frc.robot.subsystems.DuoTalonLift.Heights;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import static edu.wpi.first.wpilibj2.command.Commands.*;

import java.util.Optional;


import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;


/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {

  public final Swerve Swerve;
  public final DuoTalonLift Elevator;
  public final Coralizer Coralizer;

  private final TalonFXModule[] modules = new TalonFXModule[] 
          {
            new TalonFXModule(Constants.Swerve.FRONT_RIGHT_DRIVE_ID, Constants.Swerve.FRONT_RIGHT_TURN_ID, Constants.Swerve.FRONT_RIGHT_ABS_ENCODER_OFFSET_ROTATIONS, Constants.Swerve.FRONT_RIGHT_CANCODER_ID, 0),
            new TalonFXModule(Constants.Swerve.FRONT_LEFT_DRIVE_ID, Constants.Swerve.FRONT_LEFT_TURN_ID, Constants.Swerve.FRONT_LEFT_ABS_ENCODER_OFFSET_ROTATIONS, Constants.Swerve.FRONT_LEFT_CANCODER_ID, 1),
            new TalonFXModule(Constants.Swerve.BACK_LEFT_DRIVE_ID, Constants.Swerve.BACK_LEFT_TURN_ID, Constants.Swerve.BACK_LEFT_ABS_ENCODER_OFFSET_ROTATIONS, Constants.Swerve.BACK_LEFT_CANCODER_ID, 2),
            new TalonFXModule(Constants.Swerve.BACK_RIGHT_DRIVE_ID, Constants.Swerve.BACK_RIGHT_TURN_ID, Constants.Swerve.BACK_RIGHT_ABS_ENCODER_OFFSET_ROTATIONS, Constants.Swerve.BACK_RIGHT_CANCODER_ID, 3)
          };

  private final String[] camNames = {"limelight-left", "limelight-right"};
  private static final EnhancedCommandController driver =
      new EnhancedCommandController(0);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    Swerve = new Swerve(camNames, modules);
    Elevator = new DuoTalonLift();
    Coralizer = new Coralizer();

    // SmartDashboarding subsystems allow you to see what commands they are running
    SmartDashboard.putData("Swerve Subsystem", Swerve);

    // Configure the trigger bindings
    configureBindings();
    configureDefaultCommands();
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
    driver.rightBumper().whileTrue(Swerve.driveToNearestFeed());
    driver.b().whileTrue(Swerve.alignToReef(Optional.empty()));
    driver.y().onTrue(
        parallel(
          Coralizer.intake(),
          Swerve.rotateToNearestFeed(driver::getRequestedChassisSpeeds)
        )
    );
    driver.a().whileTrue(Coralizer.runIntakeAndCoralizer(()->0.2));
    driver.x().whileTrue(alignAndScore(Optional.empty()));
    driver.povUp().whileTrue(Elevator.goToScoringHeight());
    

    // make sure you gyro reset by aligning with the reef, not eyeballing it
    driver.back().onTrue(Swerve.resetGyro());


    driver.x().whileTrue(
      alignAndScore(Optional.empty())
    );
  }

  public void configureDefaultCommands(){
    // This is the Swerve subsystem default command, this allows the driver to drive the robot
    Swerve.setDefaultCommand
      (
        run
          (
            ()-> 
              Swerve.teleopDefaultCommand(
                driver::getRequestedChassisSpeeds,
                true
              )
              ,
              Swerve
          ).withName("Swerve Drive Command")
      );

      Elevator.setDefaultCommand(
        Elevator.setHeightLevel(Heights.Ground)
      );

      Coralizer.setDefaultCommand(
        Coralizer.runIntakeAndCoralizer(()->0)
      );

      //Gripper.setDefaultCommand(Gripper.stop());
  }

  // The command specified in here is run in autonomous
  public Command getAutonomousCommand() {
    return Swerve.sidewaysRestGyro().andThen(fourPieceLeft());
  }


  public Command fourPieceLeft() {
    return alignAndScore(Optional.of("T"))
            .andThen(
              pickUpAlignAndScore(Optional.of("K"))
            ).andThen(
              pickUpAlignAndScore(Optional.of("L"))
            ).andThen(
              pickUpAlignAndScore(Optional.of("A"))
            );
  }

  public Command pickUpAlignAndScore(Optional<String> location){
    return 
      parallel(
        Swerve.driveToNearestFeed().andThen(alignAndScore(location)),
        Coralizer.intake()
      );
  }

  public Command alignAndScore(Optional<String> location){
    return
      parallel(
        Swerve.alignToReef(location),
        
        new WaitUntilCommand(
          Swerve.atRotationSetpoint
          .and(Swerve.collisionDetected.negate())
          .and(Swerve.isCloseToDestination)
          .and(Coralizer.doneIntaking)
        ).andThen
          (
            Elevator.goToScoringHeight()
          ).until(Elevator.atSetpoint)
      )
      .andThen(Coralizer.ejectCoral().asProxy()); // asProxy because we want to be able to continue intaking while we are aligning
  }

  public Command alignAndIntake(){
    return
      deadline(
        Coralizer.intake(),
        Swerve.driveToNearestFeed()
      );
  }
}
