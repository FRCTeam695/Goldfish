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

import edu.wpi.first.math.filter.Debouncer;
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
    private boolean isSafeToRaiseElevator = true;
    private boolean hasSeenFirstBreak = true;
    public Trigger safeToRaiseElevator = new Trigger(()-> isSafeToRaiseElevator);
    public Trigger seenFirstBreak = new Trigger(()-> hasSeenFirstBreak);
    public Trigger intakeCurrentBelowThreshold;
    public Trigger isStalled;
    public Trigger debounceBeamIsMade;
    public Debouncer beambreakDebouncer;

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
        intake.getSupplyCurrent().setUpdateFrequency(20);
        intakeCurrentBelowThreshold = new Trigger(()-> ((intake.getSupplyCurrent().getValueAsDouble() < 3.0) && (intake.getSupplyCurrent().getValueAsDouble() > 0)));
        beambreakDebouncer = new Debouncer(0.07);
        debounceBeamIsMade = new Trigger(()-> !beambreakDebouncer.calculate(beamIsBroken()));
    }

    public boolean beamIsBroken(){
        return  debounceBeamIsMade.getAsBoolean();
    }

    public boolean notDecbouncedBeambreak(){
        return coralizer.getReverseLimit().getValue() == ReverseLimitValue.ClosedToGround;
    }
    
    public boolean beamNotBroken(){
    return !beamIsBroken();
    }

    public Command intake(){
        return
          (runIntakeAndCoralizer(()-> 0.6).until(this::beamIsBroken)
          .andThen(setFirstBreakStateTrue())
          .andThen(
            runIntakeAndCoralizer(()->0.4).until(intakeCurrentBelowThreshold.and(this::beamNotBroken))
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
        return runCoralizer(()-> 0.6).withTimeout(0.2).andThen(setDangerousToRaiseElevator()).andThen(setFirstBreakStateFalse());
    }

    public Command setDangerousToRaiseElevator(){
        return runOnce(()-> isSafeToRaiseElevator = false);
    }

    public Command setSafeToRaiseElevator(){
        return runOnce(()-> {
            isSafeToRaiseElevator = true;
        });
    }

    public Command setFirstBreakStateTrue(){
        return runOnce(()-> {
            hasSeenFirstBreak = true;
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
        SmartDashboard.putNumber("Coralizer speed", coralizer.getVelocity().getValueAsDouble());
        SmartDashboard.putNumber("Intake current", intake.getSupplyCurrent().getValueAsDouble());
        SmartDashboard.putNumber("Coralizer current", coralizer.getSupplyCurrent().getValueAsDouble());
        SmartDashboard.putBoolean("Debounce Testing", debounceBeamIsMade.getAsBoolean());
    }
}
