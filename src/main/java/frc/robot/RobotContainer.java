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
import frc.robot.subsystems.SideCar;
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

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

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

  private final NetworkTableInstance baseTable;

  public final Swerve swerve;
  private final DuoTalonLift elevator;
  private final Coralizer coralizer;
  private final AlgaeDislodger alagizer;
  private final Climber climber;
  private final LED led;
  
  private final SendableChooser<Command> autoChooser = new SendableChooser<>();

  public int[] reefTags = {6,7,8,9,10,11,17,18,19,20,21,22};


  private final TalonFXModule[] modules = new TalonFXModule[] 
          {
            new TalonFXModule(Constants.Swerve.FRONT_RIGHT_DRIVE_ID, Constants.Swerve.FRONT_RIGHT_TURN_ID, Constants.Swerve.FRONT_RIGHT_ABS_ENCODER_OFFSET_ROTATIONS, Constants.Swerve.FRONT_RIGHT_CANCODER_ID, 0),
            new TalonFXModule(Constants.Swerve.FRONT_LEFT_DRIVE_ID, Constants.Swerve.FRONT_LEFT_TURN_ID, Constants.Swerve.FRONT_LEFT_ABS_ENCODER_OFFSET_ROTATIONS, Constants.Swerve.FRONT_LEFT_CANCODER_ID, 1),
            new TalonFXModule(Constants.Swerve.BACK_LEFT_DRIVE_ID, Constants.Swerve.BACK_LEFT_TURN_ID, Constants.Swerve.BACK_LEFT_ABS_ENCODER_OFFSET_ROTATIONS, Constants.Swerve.BACK_LEFT_CANCODER_ID, 2),
            new TalonFXModule(Constants.Swerve.BACK_RIGHT_DRIVE_ID, Constants.Swerve.BACK_RIGHT_TURN_ID, Constants.Swerve.BACK_RIGHT_ABS_ENCODER_OFFSET_ROTATIONS, Constants.Swerve.BACK_RIGHT_CANCODER_ID, 3)
          };

  private final String[] camNames = {"limelight-left", "limelight-right"};
  private final EnhancedCommandController driver;
  
  private final SideCar sideCar;

  // The container for the robot. Contains subsystems, OI devices, and commands.
  public RobotContainer() {
    baseTable = NetworkTableInstance.getDefault();
    
    swerve = new Swerve(camNames, modules, reefTags, baseTable);
    elevator = new DuoTalonLift(baseTable);
    coralizer = new Coralizer();
    alagizer = new AlgaeDislodger(baseTable);
    climber = new Climber();
    led = new LED();


    sideCar = new SideCar();
    driver = new EnhancedCommandController(0);
    
    // SmartDashboarding subsystems allow you to see what commands they are running
    SmartDashboard.putData("Swerve Subsystem", swerve);

    // Configure the trigger bindings
    configureBindings();
    configureDefaultCommands();

    Supplier<Heights> scoringHeight = () -> sideCar.getScoringLevel();

    autoChooser.addOption("Left", 
                                alignAndScore("J", scoringHeight)
                                .andThen(
                                  pickUpAlignAndScore("K", scoringHeight)
                                )
                                .andThen(
                                  pickUpAlignAndScore("L", scoringHeight)
                                )
                                .andThen(
                                  parallel(
                                    swerve.driveToNearestFeed(),
                                    elevator.lowerToGroundThenEnd()
                                  )
                                )
                                .andThen(elevator.lowerToGroundThenEnd())
                          );
    autoChooser.addOption("Right", 
                              alignAndScore("E", scoringHeight)
                              .andThen(
                                pickUpAlignAndScore("D", scoringHeight)
                              )
                              .andThen(
                                pickUpAlignAndScore("C", scoringHeight)
                              )
                              .andThen(
                                parallel(
                                  swerve.driveToNearestFeed(),
                                  elevator.lowerToGroundThenEnd()
                                )
                              )
                              .andThen(elevator.lowerToGroundThenEnd())
    );
    autoChooser.addOption("Mid Right", alignAndScore("G", scoringHeight).andThen(elevator.lowerToGroundThenEnd()));
    autoChooser.addOption("Mid Left", alignAndScore("H", scoringHeight).andThen(elevator.lowerToGroundThenEnd()));
    SmartDashboard.putData(autoChooser);

    DataLogManager.start();
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
    swerve.isWithin10cm.and(coralizer.seenFirstBreak.negate()).and(()-> DriverStation.isAutonomous()).whileTrue(
      led.solidColor(3)
    );

    // when we are ready to shoot the coral
    coralizer.safeToRaiseElevator.and(swerve.isAtDestination).whileTrue(
      led.breatheEffect(3, 0.1)
    );

    // while auto aligning
    swerve.isFullyAutonomous.and(swerve.isAtDestination.negate()).whileTrue(led.breatheEffect(0, 0.2));

    // rotate towards the nearest feeder station
    driver.leftBumper().whileTrue(
        swerve.aimAtNearestFeedAndStrafe(driver::getRequestedChassisSpeeds)
    );

    // starts the intake
    driver.leftBumper().onTrue(
        coralizer.intake()
    );

    // starts the intake
    driver.leftTrigger().onTrue(
        coralizer.intake()
    );

    // drives to the nearest feeder station
    driver.leftTrigger().whileTrue(
        parallel(
          swerve.driveToNearestFeed(),
          elevator.lowerToGroundThenEnd()
      )
    );
    

    // make sure you gyro reset by aligning with the reef, not eyeballing it
    driver.back().onTrue(swerve.resetGyro());



    //driver.b().whileTrue(Swerve.alignToReef(Optional.empty(), ()-> Elevator.getElevatorTimeToArrival(), false));
    driver.rightBumper().onTrue(
      either(
        elevator.goToHeight(() -> sideCar.getScoringLevel()), new WaitCommand(0), coralizer.safeToRaiseElevator
      )
    );
    driver.rightBumper().onFalse(
      either(
        logTrickshotTrue().andThen(
          coralizer.ejectCoral()
                .andThen(
                  coralizer.runIntakeAndCoralizer(()-> 0).withTimeout(0.01))
                .andThen(
                  elevator.goToHeight(() -> Heights.Ground)
                )
                ).finallyDo(()-> SmartDashboard.putBoolean("Trickshot", false)),
        new WaitCommand(0), 
        coralizer.safeToRaiseElevator
        )
    );

    // driver.b().onTrue(Elevator.goToScoringHeight().until(Elevator.atSetpoint)
    //                   .andThen(Coralizer.ejectCoral()));


    //driver.b().whileTrue(Climber.climbOut(-0.1));

    // enter "climb mode"
    driver.y().whileTrue(
      parallel(
        alagizer.goToPosition(()-> Constants.Alagizer.holdRamp),
        climber.runClimb(1),
        swerve.controlToAngleAndStrafe(directionOfCages(), driver::getRequestedChassisSpeeds)
      )
    );

    driver.y().onFalse(alagizer.goToPosition(()-> Constants.Alagizer.holdRamp));

    // climbs
    driver.a().whileTrue(
      //Climber.climbInNoSoftlimits()
      parallel(
        climber.runClimb(-0.6).until(climber.closeToReverseLimit).andThen(climber.climbInNoSoftlimits()),
        alagizer.goToPosition(()-> Constants.Alagizer.holdRamp)
      )
    );
    

    driver.a().onFalse(alagizer.goToPosition(()-> Constants.Alagizer.holdRamp));

    // auto score
    driver.x().whileTrue(
      alignAndScore(sideCar.getScoringLocation().get(), () -> sideCar.getScoringLevel())
    );


    driver.x().onFalse(
      new ConditionalCommand(
        deadline(
          swerve.driveBackwards().withTimeout(0.5),
          elevator.holdHeight()
        ), 
        new WaitCommand(0),
        coralizer.safeToRaiseElevator // have not ejected the coral yet, we want to avoid a collision when we lower the elevator
      )
    );

    
    // enter "algae dislodge mode"
    driver.rightTrigger().whileTrue(
      parallel(
        alagizer.goToPosition(()-> Constants.Alagizer.safePos),
        swerve.aimAtDislodgeLocationAndStrafe(driver::getRequestedChassisSpeeds)
      )
    );

    driver.rightTrigger().onFalse(
      alagizer.goToPosition(()-> Constants.Alagizer.dislodgeAngle).until(alagizer.atSetpoint)
      .andThen(swerve.driveForwards().withTimeout(0.5))
    );

    // dumps algae/coral out of ramp
    driver.povUp().onTrue(alagizer.dump());

    // left gyro reset before auton
    driver.povLeft().onTrue(
      new ConditionalCommand(
        swerve.leftGyroReset(), 
        new WaitCommand(0), 
        ()-> DriverStation.isDisabled()
      )
    );
    
    // right gyro reset before auton
    driver.povRight().onTrue(
      new ConditionalCommand(
        swerve.rightGyroReset(), 
        //Elevator.goToScoringHeight().until(Elevator.atSetpoint).andThen(Coralizer.ejectCoral()).andThen(Coralizer.runIntakeAndCoralizer(()-> 0).withTimeout(0.01)).andThen(new WaitCommand(5)),
        new WaitCommand(0), 
        ()-> DriverStation.isDisabled()
      )
    );

    // L1 play
    driver.b().whileTrue(  
      deadline(
        coralizer.runIntakeAndCoralizer(()-> -1).withTimeout(0.2),
        alagizer.goToPosition(()-> -20.1)
      )
      .andThen(
        parallel(
          alagizer.goToPosition(()-> Constants.Alagizer.dump),
          coralizer.runIntakeAndCoralizer(()-> -1)
        )
      )
    );

    driver.b().onFalse(
      alagizer.goToPosition(()-> Constants.Alagizer.dump).until(alagizer.atSetpoint)
      .andThen(
        new WaitCommand(0.25)
      )
      .andThen(alagizer.dump())
    );

    driver.povDown().whileTrue(
      either(
        swerve.backwardsResetGyro(),
        coralizer.runIntakeAndCoralizer(()-> -0.3),
        ()-> DriverStation.isDisabled()
      )
    );


    // display all calibrated field constants on glass
    driver.leftStick().onTrue(swerve.displayVisionConstants().ignoringDisable(true));

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



  private DoubleSupplier directionOfCages() {
    return ()-> (swerve.isRedAlliance() ? -90 : 90);
  }

  public void configureDefaultCommands(){
    // This is the Swerve subsystem default command, this allows the driver to drive the robot
    swerve.setDefaultCommand
      (
        swerve.driveWithMaxSpeeds(driver::getRequestedChassisSpeeds).withName("Swerve Drive Command")
      );

      elevator.setDefaultCommand(
        elevator.goToHeight(() -> Heights.Ground)
      );

      coralizer.setDefaultCommand(
        coralizer.runIntakeAndCoralizer(()->0)
      );

      led.setDefaultCommand(led.breatheEffect(2, 2).ignoringDisable(true));

      alagizer.setDefaultCommand(alagizer.goToPosition(()-> 0));

      //Gripper.setDefaultCommand(Gripper.stop());
  }



  public Command logTrickshotTrue(){
    return runOnce(()-> {SmartDashboard.putBoolean("Trickshot", true);});
  }


  // The command specified in here is run in autonomous
  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }


  public Command pickUpAlignAndScore(String location, Supplier<Heights> scoringLevel){
    return 
      parallel(
        parallel(
          swerve.driveToNearestFeed(),
          elevator.lowerToGroundThenEnd()
        )
        .andThen(new WaitUntilCommand(coralizer.seenFirstBreak))
        .andThen(alignAndScore(location, scoringLevel)),
        coralizer.intake().asProxy()
      );
  }


  public Command alignAndScore(String location, Supplier<Heights> scoringLevel){
    return
    updateTelemetryState(1).andThen(
        // tells the elevator where is will be going later, so it can give semi-accurate time estimates for how long it will take to get there
        elevator.configureSetpoint(scoringLevel.get()).andThen(
        parallel(
          
          swerve.alignToReef(location, ()-> elevator.getElevatorTimeToArrival(), true),
          
          // all these things need 2 be true b4 it's safe to raise the elevator
          new WaitUntilCommand(
            swerve.almostAtRotationSetpoint
            .and(swerve.collisionDetected.negate())
            .and(swerve.isCloseToDestination)
            
            .and(coralizer.safeToRaiseElevator)
          ).andThen(
            updateTelemetryState(2)
          ).andThen
            (
              elevator.goToHeight(scoringLevel)
            ).until(elevator.atSetpoint)
        ))
        .andThen(
          updateTelemetryState(3)
        )
      .andThen(
            coralizer.ejectCoral().asProxy() // asProxy because we want to be able to continue intaking while we are aligning
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
