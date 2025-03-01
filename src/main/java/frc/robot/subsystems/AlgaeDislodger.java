package frc.robot.subsystems;
import com.ctre.phoenix6.hardware.TalonFXS;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class AlgaeDislodger extends SubsystemBase{
    public SparkMax neo550;
    public TalonFXS talon;

    public AlgaeDislodger() {
        neo550 = new SparkMax(0, MotorType.kBrushless); //change ID accordingly
        talon = new TalonFXS(0); //again, change ID accordingly
    }

    //may need to put this in a command rather than periodic method
    @Override
    public void periodic() {
        SmartDashboard.putNumber("NEO 550 Motor Position from Encoder", neo550.getAbsoluteEncoder().getPosition());
    }
}
