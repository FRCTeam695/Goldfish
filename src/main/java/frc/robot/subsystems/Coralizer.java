package frc.robot.subsystems;


import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXSConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.hardware.TalonFXS;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorArrangementValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.ReverseLimitValue;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
//import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class Coralizer extends SubsystemBase{
    //DigitalInput beamBreak;
    private TalonFXS coralizer;
    private TalonFXS intake;
    public double startTime = 0;
    private boolean isSafeToRaiseElevator = true;
    private boolean hasSeenFirstBreak = true;
    public Trigger safeToRaiseElevator = new Trigger(()-> isSafeToRaiseElevator);
    public Trigger seenFirstBreak = new Trigger(()-> hasSeenFirstBreak);
    public Trigger beenEnoughTime = new Trigger(this::timeExpired);
    public Trigger isStalled;

    public Coralizer(){
        //beamBreak = new DigitalInput(0);
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

        isStalled = new Trigger(()-> intake.getMotorStallCurrent().getValueAsDouble() > 3);
    }

    public boolean beamIsBroken(){
        return  coralizer.getReverseLimit().getValue() == ReverseLimitValue.ClosedToGround;
    }

    public boolean timeExpired(){
        double timeDiff = Math.abs(Timer.getFPGATimestamp() - startTime);
        SmartDashboard.putNumber("CORALIZER time difference", timeDiff);
        return (timeDiff >= 0.65);
    }
    
    public boolean beamNotBroken(){
    return !beamIsBroken();
    }

    public Command intake(){
        return
          (runIntakeAndCoralizer(()-> 0.6).until(this::beamIsBroken)
          .andThen(setFirstBreakStateTrue())
          .andThen(
            runIntakeAndCoralizer(()->0.4).until(beenEnoughTime.and(this::beamNotBroken))
          )
          .andThen(
              setSafeToRaiseElevator()
            .andThen(runIntakeAndCoralizer(()-> -0.1).until(this::beamIsBroken))
            .andThen(runIntakeAndCoralizer(()-> 0))
          )).withName("intake");
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
        return run(
            () -> {
                coralizer.set(speed.getAsDouble());
                intake.set(0);
            }
        );
    }

    public Command ejectCoral(){
        return runCoralizer(()-> 0.6).withTimeout(0.3).andThen(setDangerousToRaiseElevator()).andThen(setFirstBreakStateFalse());
    }

    public Command setDangerousToRaiseElevator(){
        return runOnce(()-> isSafeToRaiseElevator = false);
    }

    public Command setSafeToRaiseElevator(){
        return runOnce(()-> {
            isSafeToRaiseElevator = true;
            SmartDashboard.putNumber("Beambreak Time", Timer.getFPGATimestamp() - startTime);
        });
    }

    public Command setFirstBreakStateTrue(){
        return runOnce(()-> {
            hasSeenFirstBreak = true;
            startTime = Timer.getFPGATimestamp();
        });
    }

    public Command setFirstBreakStateFalse(){
        return runOnce(()-> hasSeenFirstBreak = false);
    }

    public Command requireSubsystem(){
        return new WaitCommand(0);
    }

    @Override
    public void periodic(){
        SmartDashboard.putBoolean("Beambreak", beamIsBroken());
        SmartDashboard.putBoolean("Has Finished Intaking", safeToRaiseElevator.getAsBoolean());
        SmartDashboard.putBoolean("Has Seen First Break", hasSeenFirstBreak);
        SmartDashboard.putBoolean("CORALIZER Been enough time", beenEnoughTime.getAsBoolean());
        SmartDashboard.putNumber("Intake current", intake.getSupplyCurrent().getValueAsDouble());
        SmartDashboard.putNumber("Coralizer current", coralizer.getSupplyCurrent().getValueAsDouble());
        SmartDashboard.putNumber("CORALIZER Start time", startTime);
    }
}
