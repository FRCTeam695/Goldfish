package frc.robot.subsystems;


import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXSConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.hardware.TalonFXS;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorArrangementValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
//import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Coralizer extends SubsystemBase{
    //DigitalInput beamBreak;
    private TalonFXS coralizer;
    private TalonFXS intake;
    private DigitalInput beamBreak;

    public Coralizer(){
        //beamBreak = new DigitalInput(0);
        coralizer = new TalonFXS(52);
        intake = new TalonFXS(53);

        
        TalonFXSConfiguration config = new TalonFXSConfiguration();
        config.CurrentLimits.SupplyCurrentLimit = 20.0;
        config.CurrentLimits.SupplyCurrentLimitEnable = true;
        config.Commutation.MotorArrangement = MotorArrangementValue.NEO550_JST;

        MotorOutputConfigs motorOutputConfigs = new MotorOutputConfigs();
        motorOutputConfigs.NeutralMode = NeutralModeValue.Brake;
        motorOutputConfigs.Inverted = InvertedValue.Clockwise_Positive;

        config.MotorOutput = motorOutputConfigs;

        coralizer.getConfigurator().apply(config);
        intake.getConfigurator().apply(config);

        beamBreak = new DigitalInput(6);
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
        )
        .andThen(
            runIntakeAndCoralizer(()->-0.2).until(this::beamIsBroken)
        ).andThen(runIntakeAndCoralizer(()-> 0));
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
        //DutyCycleOut coralizerOut = new DutyCycleOut(0);
        return run(
            () -> {
                // coralizerOut.Output = speed.getAsDouble();
                // coralizer.setControl(coralizerOut);
                coralizer.set(speed.getAsDouble());
            }
        );
    }

    public Command ejectCoral(){
        return runCoralizer(()-> 0.6).withTimeout(0.2);
    }

    @Override
    public void periodic(){
        SmartDashboard.putBoolean("Beambreak", beamIsBroken());
    }
}
