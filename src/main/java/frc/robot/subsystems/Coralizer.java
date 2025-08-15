package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXSConfiguration;
import com.ctre.phoenix6.controls.ControlRequest;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.hardware.TalonFXS;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorArrangementValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
//import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Coralizer extends SubsystemBase {
    private DigitalInput beamBreak;
    private TalonFXS coralizer;
    private TalonFXS intake;

    private double SupplyCurrent;

    double encoder;

    public Coralizer() {

        beamBreak = new DigitalInput(9);
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

        // coralizer.getPosition().setUpdateFrequency(50);
        intake.getSupplyCurrent().setUpdateFrequency(50);
        SupplyCurrent = intake.getSupplyCurrent().getValueAsDouble();

        coralizer.setPosition(0.0);
        coralizer.getSupplyCurrent().setUpdateFrequency(50);

    }

    public Command detectEncoderChange() {
        return new FunctionalCommand(
                () -> {
                    //coralizer.setPosition(0.0);
                    encoder = coralizer.getPosition().getValueAsDouble();
                    intake.set(0.5);
                },
                () -> {
                    
                    System.out.println("1 E " + coralizer.getPosition().getValueAsDouble());

                },
                (interrupted) -> {
                    intake.set(0.0);
                    coralizer.set(0.0);
                    coralizer.setPosition(0.0);
                },
                () -> {
                    if (coralizer.getPosition().getValueAsDouble() - encoder >= 1.0)
                        return true;
                    return false;
                },
                this);
    }

    public Command moveCoralizerToElevator() {
        return new FunctionalCommand(
                () -> {
                    coralizer.setPosition(0.0);
                    intake.set(0.5);
                    coralizer.set(0.5);
                },
                () -> {
                    System.out.println("2 E " + coralizer.getPosition().getValueAsDouble());
                },
                (interrupted) -> {
                    coralizer.set(0.0);
                    intake.set(0.0);
                    coralizer.setPosition(0.0);
                },
                () -> {
                    if (coralizer.getPosition().getValueAsDouble() >= 21.25 )
                        return true;
                    return false;
                },
                this);
    }

    public Command recordEncoder() {
        return run(() -> {
            SmartDashboard.putNumber("Coralizer encoder", coralizer.getPosition().getValueAsDouble());
            SmartDashboard.putBoolean("beambreak", beamBreak.get());
        });
    }

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
                });
    }

    @Override
    public void periodic() {

        SmartDashboard.putNumber("Coralizer speed", coralizer.getVelocity().getValueAsDouble());
        SmartDashboard.putNumber("Intake current", intake.getSupplyCurrent().getValueAsDouble());
        SmartDashboard.putNumber("Coralizer current", coralizer.getSupplyCurrent().getValueAsDouble());

    }
}
