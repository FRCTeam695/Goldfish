package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;

import java.util.function.DoubleSupplier;

import com.revrobotics.spark.SparkLowLevel;

//import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Coralizer extends SubsystemBase{
    //DigitalInput beamBreak;
    public SparkMax coralizer;
    public SparkMax intake;
    public Coralizer(){
        //beamBreak = new DigitalInput(0);
        coralizer = new SparkMax(52, SparkLowLevel.MotorType.kBrushless);
    }

    public Command runCoralizer(DoubleSupplier speed) {
        return run(
            () -> {
                coralizer.set(speed.getAsDouble());
            }
        );
    }
}
