// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;

// import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.BisonLib.BaseProject.Controller.EnhancedCommandController;

// import frc.robot.Subsystems.CoralGripper2Motors;
import frc.robot.subsystems.Swerve;
import frc.BisonLib.BaseProject.Swerve.Modules.TalonFXModule;

import static edu.wpi.first.wpilibj2.command.Commands.*;

import java.util.Optional;


public class RobotContainer {

  public final Swerve Swerve;
  //public final CoralGripper2Motors Gripper;
  //public final FakeElevator FakeElevator;
  //public final AlgaeGripper AlgaeGripper;

  //public final CoralGripper2Motors Gripper2;

  // Creates an array that holds all our pose estimation cameras, pass this into Swerve
  private final String[] camNames = {"limelight-left", "limelight-right"};//robot left and robot right
    
  // Creates an array of all the swerve modules, pass this into Swerve
  private final TalonFXModule[] modules = new TalonFXModule[] 
          {
            new TalonFXModule(Constants.Swerve.FRONT_RIGHT_DRIVE_ID, Constants.Swerve.FRONT_RIGHT_TURN_ID, Constants.Swerve.FRONT_RIGHT_ABS_ENCODER_OFFSET_ROTATIONS, Constants.Swerve.FRONT_RIGHT_CANCODER_ID, 0),
            new TalonFXModule(Constants.Swerve.FRONT_LEFT_DRIVE_ID, Constants.Swerve.FRONT_LEFT_TURN_ID, Constants.Swerve.FRONT_LEFT_ABS_ENCODER_OFFSET_ROTATIONS, Constants.Swerve.FRONT_LEFT_CANCODER_ID, 1),
            new TalonFXModule(Constants.Swerve.BACK_LEFT_DRIVE_ID, Constants.Swerve.BACK_LEFT_TURN_ID, Constants.Swerve.BACK_LEFT_ABS_ENCODER_OFFSET_ROTATIONS, Constants.Swerve.BACK_LEFT_CANCODER_ID, 2),
            new TalonFXModule(Constants.Swerve.BACK_RIGHT_DRIVE_ID, Constants.Swerve.BACK_RIGHT_TURN_ID, Constants.Swerve.BACK_RIGHT_ABS_ENCODER_OFFSET_ROTATIONS, Constants.Swerve.BACK_RIGHT_CANCODER_ID, 3)
          };

  // Creates an Xbox Controller Object, the first parameter relates to the port on the laptop
  public static final EnhancedCommandController Driver = new EnhancedCommandController(0);

  // Creates joystick controller objects for the coral grippers
  // private final CommandJoystick m_joystickIntakeLeft = new CommandJoystick(0);
  // joystick for intake is on the left of the table
  // private final CommandJoystick m_joystickShootRight = new CommandJoystick(1);
  // joystick for shooting is on the right of the table
  //UsbCamera usbCamera = new UsbCamera("USB Camera 0", 0);
  //MjpegServer mjpegServer1 = new MjpegServer("serve_USB Camera 0", 1181);

  //Creates joystick controller objects for the algae grippers



  private SendableChooser<Command> autoChooser;
  public RobotContainer() {
    Swerve = new Swerve(camNames, modules);
    //Gripper1 = new CoralGripper1Motor();
    //FakeElevator = new FakeElevator();
    //AlgaeGripper = new AlgaeGripper();

    //Gripper = new CoralGripper2Motors();
    //CameraServer.startAutomaticCapture();

    // configureBindings binds Commands to different button presses (or triggers),
    // Commands are an important part of our programming, here are the docs,
    // https://docs.wpilib.org/en/2021/docs/software/commandbased/index.html
    // I would highly suggest reading all of the links on this page
    configureBindings();

    // configureDefaultCommands sets default commands to each subsystem, 
    // a default command runs when the subsystem is not otherwise required by a different command, look at the docs for more details,
    // https://docs.wpilib.org/en/2021/docs/software/old-commandbased/commands/default-commands.html
    configureDefaultCommands();
    sendAutoChooserToDashboard();

    // SmartDashboarding subsystems allow you to see what commands they are running
    SmartDashboard.putData("Swerve Subsystem", Swerve);
  }

  /*
   * I put it in a method so I can't accidentally comment out the auton
   */
  private void sendAutoChooserToDashboard(){
    // This creates our auto chooser and sends it to SmartDashboard, look at pathplanner docs for more details
    // https://pathplanner.dev/home.html
    autoChooser = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData("Auto Chooser", autoChooser);
  }

  private void configureBindings() {
    Driver.back().onTrue(Swerve.resetGyro());

    // comment out drive and odometry code before running this
    Driver.a().onTrue(Swerve.runWheelCharacterization());
    Driver.b().onTrue(Swerve.backwardsResetGyro());
    Driver.leftBumper().whileTrue(Swerve.alignToReef(Optional.empty()));
    Driver.rightBumper().whileTrue(Swerve.driveToNearestFeed());
    Driver.y().whileTrue(
      fourPieceLeft()
    );
    //Driver.b().whileTrue(Gripper.intake());
  }

  public void configureDefaultCommands(){
    // This is the Swerve subsystem default command, this allows the driver to drive the robot
    Swerve.setDefaultCommand
      (
        run
          (
            ()-> 
              Swerve.teleopDefaultCommand(
                Driver::getRequestedChassisSpeeds,
                true
              )
              ,
              Swerve
          ).withName("Swerve Drive Command")
      );

      //Gripper.setDefaultCommand(Gripper.stop());
  }

  // The command specified in here is run in autonomous
  public Command getAutonomousCommand() {
    return Swerve.backwardsResetGyro().andThen(fourPieceLeft());
  }


  public Command fourPieceLeft() {
    return Swerve.alignToReef(Optional.of("J"))
            .andThen(Swerve.driveToNearestFeed())
            .andThen(Swerve.alignToReef(Optional.of("K")))
            .andThen(Swerve.driveToNearestFeed())
            .andThen(Swerve.alignToReef(Optional.of("L")))
            .andThen(Swerve.driveToNearestFeed())
            .andThen(Swerve.alignToReef(Optional.of("A")));
  }
}
