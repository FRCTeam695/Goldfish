

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.subsystems.Coralizer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;


public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  
  private final Coralizer coralizer = new Coralizer();

  // Replace with CommandPS4Controller or CommandJoystick if needed

  private final CommandXboxController m_driverController =
        new CommandXboxController(OperatorConstants.kDriverControllerPort);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the trigger bindings
    configureBindings();
  }

 
  private void configureBindings() {
    m_driverController.b().whileTrue(
      coralizer.detectEncoderChange()
    );
    coralizer.setDefaultCommand(coralizer.stopIntakeAndCoralizer());
  }

  
  // public Command getAutonomousCommand() {
  //   // An example command will be run in autonomous
  //   return Autos.exampleAuto(m_exampleSubsystem);
  // }
}
