package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXSConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.hardware.TalonFXS;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorArrangementValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
//import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Coralizer extends SubsystemBase {
    // DigitalInput beamBreak;
    private TalonFXS coralizer;
    private TalonFXS intake;

    private double SupplyCurrent;

    public Coralizer() {
        // beamBreak = new DigitalInput(0);
        coralizer = new TalonFXS(52);
        intake = new TalonFXS(53);

        TalonFXSConfiguration config = new TalonFXSConfiguration();
        config.CurrentLimits.SupplyCurrentLimit = 20.0;
        config.HardwareLimitSwitch.ForwardLimitEnable = false;
        config.HardwareLimitSwitch.ReverseLimitEnable = false;
        config.CurrentLimits.SupplyCurrentLimitEnable = true;
        config.Commutation.MotorArrangement = MotorArrangementValue.NEO550_JST;

        MotorOutputConfigs motorOutputConfigs = new MotorOutputConfigs();
        motorOutputConfigs.NeutralMode = NeutralModeValue.Brake;
        motorOutputConfigs.Inverted = InvertedValue.Clockwise_Positive;

        config.MotorOutput = motorOutputConfigs;

        coralizer.getConfigurator().apply(config);
        intake.getConfigurator().apply(config);

        intake.getSupplyCurrent().setUpdateFrequency(20);
        SupplyCurrent = intake.getSupplyCurrent().getValueAsDouble();

    }

    public Command runIntakeAndCoralizer(DoubleSupplier speed){
    DutyCycleOut output = new DutyCycleOut(0);
    return run(()->{
    output.Output = speed.getAsDouble();

    coralizer.setControl(output);
    intake.setControl(output);

    
    });
    }

    // public Command runIntakeAndCoralizer(DoubleSupplier speed) {
    //     return new FunctionalCommand(() -> {
    //         DutyCycleOut output = new DutyCycleOut(0);
    //     }, () -> {
    //         DutyCycleOut output = new DutyCycleOut(0);
    //         output.Output = speed.getAsDouble();

    //         coralizer.setControl(output);
    //         intake.setControl(output);
    //     },
        
    //     interrupted -> {
            
    //     },
            
    //         () -> false, 
            
            
    //         this);
    // }

    public Command runIntakeMotor(DoubleSupplier speed) {
        DutyCycleOut output = new DutyCycleOut(0);
        return run(() -> {
            output.Output = speed.getAsDouble();
            intake.setControl(output);
        });
    }

    public Command runCoralizer(DoubleSupplier speed) {
        return run(
                () -> {
                    coralizer.set(speed.getAsDouble());
                    intake.set(0);
                });
    }

    @Override
    public void periodic() {

        SmartDashboard.putNumber("Coralizer speed", coralizer.getVelocity().getValueAsDouble());
        SmartDashboard.putNumber("Intake current", intake.getSupplyCurrent().getValueAsDouble());
        SmartDashboard.putNumber("Coralizer current", coralizer.getSupplyCurrent().getValueAsDouble());

    }
}
