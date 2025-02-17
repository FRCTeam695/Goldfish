package frc.robot.subsystems;


import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.DigitalInput;
//import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Coralizer extends SubsystemBase{
    //DigitalInput beamBreak;
    private TalonFX coralizer;
    private TalonFX intake;
    private DigitalInput beamBreak;
    public Coralizer(){
        //beamBreak = new DigitalInput(0);
        coralizer = new TalonFX(52);
        intake = new TalonFX(54);


        MotorOutputConfigs coralizerConfig = new MotorOutputConfigs();
        coralizerConfig.NeutralMode = NeutralModeValue.Brake;
        coralizerConfig.Inverted = InvertedValue.Clockwise_Positive;
        coralizer.getConfigurator().apply(coralizerConfig);

        MotorOutputConfigs intakeConfig = new MotorOutputConfigs();
        intakeConfig.NeutralMode = NeutralModeValue.Brake;

        coralizerConfig.Inverted = InvertedValue.Clockwise_Positive;
        intake.getConfigurator().apply(intakeConfig);

        beamBreak = new DigitalInput(0);
    }

    public boolean beamIsBroken(){
        return beamBreak.get();
      }
    
    public boolean beamNotBroken(){
    return !beamIsBroken();
    }

    public Command intake(){
        return 
        runIntakeAndCoralizer(()-> 0.6).until(this::beamIsBroken)
        .andThen(
            runIntakeAndCoralizer(()->0.6).until(this::beamNotBroken)
        );
    }

    public Command runIntakeAndCoralizer(DoubleSupplier speed){
        DutyCycleOut output = new DutyCycleOut(0);
        return run(()->{
            output.Output = speed.getAsDouble();
            coralizer.setControl(output);
            intake.setControl(output);
        });
    }

    public Command runIntakeMotor(DoubleSupplier speed){
        DutyCycleOut output = new DutyCycleOut(0);
        return run(()->{
            output.Output = speed.getAsDouble();
            intake.setControl(output);
        });
    }

    public Command runCoralizer(DoubleSupplier speed) {
        DutyCycleOut coralizerOut = new DutyCycleOut(0);
        return run(
            () -> {
                coralizerOut.Output = speed.getAsDouble();
                coralizer.setControl(coralizerOut);
            }
        );
    }

    public Command ejectCoral(){
        return runCoralizer(()-> 0.6).withTimeout(0.2);
    }
}
