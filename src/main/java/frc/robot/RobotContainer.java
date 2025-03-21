// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.BisonLib.BaseProject.Controller.EnhancedCommandController;

// import frc.robot.Subsystems.CoralGripper2Motors;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.AlgaeDislodger;
import frc.robot.subsystems.Coralizer;
import frc.BisonLib.BaseProject.Swerve.Modules.TalonFXModule;

import frc.robot.subsystems.DuoTalonLift;
import frc.robot.subsystems.DuoTalonLift.Heights;
import frc.robot.subsystems.LED;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import static edu.wpi.first.wpilibj2.command.Commands.*;

import java.util.Optional;

import edu.wpi.first.networktables.IntegerSubscriber;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
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
  public final AlgaeDislodger Alagizer;
  public IntegerSubscriber scoringHeight;
  public static final LED led = new LED();
  SendableChooser<Command> autoChooser = new SendableChooser<>();


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
    Alagizer = new AlgaeDislodger();
    scoringHeight = NetworkTableInstance.getDefault().getTable("sidecarTable").getIntegerTopic("scoringLevel").subscribe(1);

    // SmartDashboarding subsystems allow you to see what commands they are running
    SmartDashboard.putData("Swerve Subsystem", Swerve);

    // Configure the trigger bindings
    configureBindings();
    configureDefaultCommands();


    autoChooser.addOption("Left", 
                                alignAndScore(Optional.of("J"))
                                .andThen(
                                  pickUpAlignAndScore(Optional.of("K"))
                                )
                                .andThen(
                                  pickUpAlignAndScore(Optional.of("L"))
                                )
                                .andThen(
                                  Elevator.setHeightLevel(Heights.Ground)
                                ).until(Elevator.atSetpoint)
                          );
    autoChooser.addOption("Right", 
                              alignAndScore(Optional.of("E"))
                              .andThen(
                                pickUpAlignAndScore(Optional.of("D"))
                              )
                              .andThen(
                                pickUpAlignAndScore(Optional.of("C"))
                              )
                              .andThen(
                                Elevator.setHeightLevel(Heights.Ground)
                              ).until(Elevator.atSetpoint)
    );
    SmartDashboard.putData(autoChooser);
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

    Coralizer.seenFirstBreak.negate().and(Swerve.isWithin10cm).and(()-> DriverStation.isAutonomous()).onTrue(
      led.solidColor(3)
    );

    //Coralizer.isStalled.whileTrue(led.breatheEffect(4, 0.1));

    
    driver.rightBumper().onTrue(
      logTrickshotTrue().andThen(
      Coralizer.ejectCoral()
            .andThen(
              Coralizer.runIntakeAndCoralizer(()-> 0).withTimeout(0.01))
            .andThen(Elevator.setHeightLevel(Heights.Ground))
            ).finallyDo(()-> SmartDashboard.putBoolean("Trickshot", false)));
    
    driver.leftBumper().whileTrue(
        Swerve.rotateToNearestFeed(driver::getRequestedChassisSpeeds)
    );

    driver.leftBumper().onTrue(
        intake()
    );
    

    // make sure you gyro reset by aligning with the reef, not eyeballing it
    driver.back().onTrue(Swerve.resetGyro());

    driver.a().whileTrue(
        Coralizer.runIntakeAndCoralizer(()-> -0.1)
    );

    //driver.b().whileTrue(Swerve.alignToReef(Optional.empty(), ()-> Elevator.getElevatorTimeToArrival(), false));
    driver.b().onTrue(
      Elevator.goToScoringHeight()
    );

    driver.y().whileTrue(Swerve.driveBackwards());

    driver.x().whileTrue(
      alignAndScore(Optional.empty())
    );
    
    driver.rightTrigger().toggleOnTrue(
      parallel(
        Alagizer.goToPosition(()-> Constants.Alagizer.dislodgeAngle),
        Swerve.rotateToDislodgeLocation(driver::getRequestedChassisSpeeds)
      )
    );

    driver.povUp().onTrue(Alagizer.dump());

    driver.povLeft().onTrue(
      Swerve.leftGyroReset()
      );
    
    driver.povRight().onTrue(
      Swerve.rightGyroReset()
    );

    driver.leftTrigger().toggleOnTrue(
      Swerve.rotateToReefCenter(driver::getRequestedChassisSpeeds)
    );

    driver.leftStick().onTrue(Swerve.displayVisionConstants().ignoringDisable(true));
  }

  public void configureDefaultCommands(){
    // This is the Swerve subsystem default command, this allows the driver to drive the robot
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

      led.setDefaultCommand(led.breatheEffect(2, 2).ignoringDisable(true));

      Alagizer.setDefaultCommand(Alagizer.goToPosition(()-> 0));

      //Gripper.setDefaultCommand(Gripper.stop());
  }

  public Command intake(){
    return
      Coralizer.runIntakeAndCoralizer(()-> 0.6).until(Coralizer::beamIsBroken)
      .andThen(Coralizer.setFirstBreakStateTrue())
      .andThen(
        Coralizer.runIntakeAndCoralizer(()->0.4).until(Coralizer::beamNotBroken)
      )
      .andThen(
          Coralizer.setSafeToRaiseElevator()
        .andThen(Coralizer.runIntakeAndCoralizer(()-> -0.1).until(Coralizer::beamIsBroken))
        .andThen(Coralizer.runIntakeAndCoralizer(()-> 0))
      );

}

  public Command logTrickshotTrue(){
    return runOnce(()-> {SmartDashboard.putBoolean("Trickshot", true);});
  }


  // The command specified in here is run in autonomous
  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }


  public Command pickUpAlignAndScore(Optional<String> location){
    return 
      parallel(
        parallel(
          Swerve.driveToNearestFeed(),
          Elevator.setHeightLevel(Heights.Ground).until(Elevator.atSetpoint)
        )
        .andThen(new WaitUntilCommand(Coralizer.seenFirstBreak))
        .andThen(alignAndScore(location)),
        intake().asProxy()
      );
  }


  public Command alignAndScore(Optional<String> location){
    return
    updateTelemetryState(1).andThen(
        Elevator.configureSetpoint().andThen(
        parallel(
          // tells the elevator where is will be going later, 
          // so it can give semi-accurate time estimates for how long it will take to get there
          
          Swerve.alignToReef(location, ()-> Elevator.getElevatorTimeToArrival(), true),
          
          new WaitUntilCommand(
            Swerve.almostAtRotationSetpoint
            .and(Swerve.collisionDetected.negate())
            .and(Swerve.isCloseToDestination)
            
            .and(Coralizer.safeToRaiseElevator)
          ).andThen(
            updateTelemetryState(2)
          ).andThen
            (
              Elevator.goToScoringHeight()
            ).until(Elevator.atSetpoint)
        ))
        .andThen(
          updateTelemetryState(3)
        )
      .andThen(
            Coralizer.ejectCoral().asProxy() // asProxy because we want to be able to continue intaking while we are aligning
        )
      .andThen(
        updateTelemetryState(4)
      )
    ).withName("AlignAndScore");
  }


  private Command updateTelemetryState(int state) {
    return runOnce(()-> SmartDashboard.putNumber("Align State", state));
  }

  public Command alignAndIntake(){
    return
      deadline(
        intake(),
        Swerve.driveToNearestFeed()
      );
  }
}
