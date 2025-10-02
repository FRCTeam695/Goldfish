// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.BisonLib.BaseProject.Controller.EnhancedCommandController;

// import frc.robot.Subsystems.CoralGripper2Motors;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.AlgaeDislodger;
import frc.robot.subsystems.Coralizer;
import frc.robot.subsystems.Climber;
import frc.BisonLib.BaseProject.Swerve.Modules.TalonFXModule;

import frc.robot.subsystems.DuoTalonLift;
import frc.robot.subsystems.DuoTalonLift.Heights;
import frc.robot.subsystems.LED;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import static edu.wpi.first.wpilibj2.command.Commands.*;

import java.util.Optional;


import edu.wpi.first.networktables.IntegerSubscriber;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DataLogManager;
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
  public final Climber Climber;
  public IntegerSubscriber scoringHeight;
  public final LED led = new LED();
  SendableChooser<Command> autoChooser = new SendableChooser<>();

  public int[] reefTags = {6,7,8,9,10,11,17,18,19,20,21,22};


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
    Swerve = new Swerve(camNames, modules, reefTags);
    Elevator = new DuoTalonLift();
    Coralizer = new Coralizer();
    Alagizer = new AlgaeDislodger();
    Climber = new Climber();
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
                                  pickUpAlignAndScore(Optional.of("M"))
                                )
                                .andThen(
                                  parallel(
                                    Swerve.driveToNearestFeed(),
                                    Elevator.setHeightLevel(Heights.Ground).until(Elevator.atSetpoint)
                                  )
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
                                pickUpAlignAndScore(Optional.of("B"))
                              )
                              .andThen(
                                parallel(
                                  Swerve.driveToNearestFeed(),
                                  Elevator.setHeightLevel(Heights.Ground).until(Elevator.atSetpoint)
                                )
                              )
                              .andThen(
                                Elevator.setHeightLevel(Heights.Ground)
                              ).until(Elevator.atSetpoint)
    );
    autoChooser.addOption("Mid Right", alignAndScore(Optional.of("G")).andThen(Elevator.setHeightLevel(Heights.Ground).until(Elevator.atSetpoint)));
    autoChooser.addOption("Mid Left", alignAndScore(Optional.of("H")).andThen(Elevator.setHeightLevel(Heights.Ground).until(Elevator.atSetpoint)));
    SmartDashboard.putData(autoChooser);

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
    Swerve.isWithin10cm.and(Coralizer.seenFirstBreak.negate()).and(()-> DriverStation.isAutonomous()).whileTrue(
      led.solidColor(3)
    );

    // when we are ready to shoot the coral
    Coralizer.safeToRaiseElevator.and(Swerve.isAtDestination).whileTrue(
      led.breatheEffect(3, 0.1)
    );

    // while auto aligning
    Swerve.isFullyAutonomous.and(Swerve.isAtDestination.negate()).whileTrue(led.breatheEffect(0, 0.2));

    // rotate towards the nearest feeder station
    driver.leftBumper().whileTrue(
        Swerve.rotateToNearestFeed(driver::getRequestedChassisSpeeds)
    );

    // starts the intake
    driver.leftBumper().onTrue(
        Coralizer.intake().andThen(new WaitCommand(0.25).andThen(Coralizer.setSafeToRaiseElevator()))
    );

    // starts the intake
    driver.leftTrigger().onTrue(
        Coralizer.intake()
    );

    // drives to the nearest feeder station
    driver.leftTrigger().whileTrue(
        parallel(
          Swerve.driveToNearestFeed(),
          Elevator.setHeightLevel(Heights.Ground).until(Elevator.atSetpoint)
      )
    );
    

    // make sure you gyro reset by aligning with the reef, not eyeballing it
    driver.back().onTrue(Swerve.resetGyro());



    //driver.b().whileTrue(Swerve.alignToReef(Optional.empty(), ()-> Elevator.getElevatorTimeToArrival(), false));
    driver.rightBumper().onTrue(
      parallel(
        either(Elevator.goToScoringHeight(), new WaitCommand(0), Coralizer.safeToRaiseElevator).withTimeout(0.75)
      )
      .andThen(either(logTrickshotTrue()
                        .andThen(Coralizer.ejectCoral()
                        .andThen(Coralizer.runIntakeAndCoralizer(()-> 0).withTimeout(0.01))
                        .andThen(Elevator.setHeightLevel(Heights.Ground))
                        ).finallyDo(()-> SmartDashboard.putBoolean("Trickshot", false)),
        new WaitCommand(0), 
        Coralizer.safeToRaiseElevator
        ))
    );

    // enter "climb mode"
    driver.y().whileTrue(
      parallel(
        Alagizer.goToPosition(()-> Constants.Alagizer.holdRamp),
        Climber.runClimb(1),
        Swerve.rotateToAngle(()-> (Swerve.isRedAlliance() ? -90 : 90), driver::getRequestedChassisSpeeds)
      )
    );

    driver.y().onFalse(Alagizer.goToPosition(()-> Constants.Alagizer.holdRamp));

    // climbs
    driver.a().whileTrue(
      //Climber.climbInNoSoftlimits()
      parallel(
        Climber.runClimb(-0.6).until(Climber.closeToReverseLimit).andThen(Climber.climbInNoSoftlimits()),
        Alagizer.goToPosition(()-> Constants.Alagizer.holdRamp)
      )
    );
    

    driver.a().onFalse(Alagizer.goToPosition(()-> Constants.Alagizer.holdRamp));


    // auto score
    driver.x().whileTrue(
      //Swerve.alignToReef(Optional.of("A"), ()-> Elevator.getElevatorTimeToArrival(), false)
      alignAndScore(Optional.empty())
    );


    driver.x().onFalse(
      new ConditionalCommand(
        deadline(
          Swerve.driveBackwards().withTimeout(0.5),
          Elevator.holdHeight()
        ), 
        new WaitCommand(0),
        Coralizer.safeToRaiseElevator // have not ejected the coral yet, we want to avoid a collision when we lower the elevator
      )
    );

    
    // enter "algae dislodge mode"
    driver.rightTrigger().whileTrue(
      parallel(
        Alagizer.goToPosition(()-> Constants.Alagizer.safePos),
        Swerve.rotateToDislodgeLocation(driver::getRequestedChassisSpeeds)
      )
    );

    driver.rightTrigger().onFalse(
      Alagizer.goToPosition(()-> Constants.Alagizer.dislodgeAngle).until(Alagizer.atSetpoint)
      .andThen(Swerve.driveForwards().withTimeout(0.5))
    );

    // dumps algae/coral out of ramp
    driver.povUp().onTrue(Alagizer.dump());

    // left gyro reset before auton
    driver.povLeft().onTrue(
      new ConditionalCommand(
        Swerve.leftGyroReset(), 
        new WaitCommand(0), 
        ()-> DriverStation.isDisabled()
      )
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

    // L1 play
    driver.b().onTrue(  
        parallel(
        Coralizer.runIntakeAndCoralizerNoStop(()-> -1)
        .withTimeout(0.6)
        // .andThen(Alagizer.goToPosition(()-> -20.1))
        // .andThen(
        //   parallel(
        //    Alagizer.goToPosition(()-> Constants.Alagizer.dump),
        //     Coralizer.runIntakeAndCoralizer(()-> -1)
        //   )
      ).andThen(
        Alagizer.goToPosition(()-> Constants.Alagizer.dump).until(Alagizer.atSetpoint))
        .andThen(
          new WaitCommand(0.25) 
        )
        .andThen(Alagizer.dump())
    );

    // driver.b().onFalse(
      
    // );

    driver.povDown().whileTrue(
      either(
        Swerve.backwardsResetGyro(),
        Coralizer.runIntakeAndCoralizer(()-> -0.3),
        ()-> DriverStation.isDisabled()
      )
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
        Coralizer.intake().asProxy()
      );
  }


  public Command alignAndScore(Optional<String> location){
    return
    updateTelemetryState(1).andThen(
        // tells the elevator where is will be going later, so it can give semi-accurate time estimates for how long it will take to get there
        Elevator.configureSetpoint().andThen(
        parallel(
          
          Swerve.alignToReef(location, ()-> Elevator.getElevatorTimeToArrival(), true),
          
          // all these things need 2 be true b4 it's safe to raise the elevator
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
}