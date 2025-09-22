package frc.robot.subsystems;


import static edu.wpi.first.wpilibj2.command.Commands.either;

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
import edu.wpi.first.networktables.IntegerSubscriber;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.wpilibj.DigitalInput;
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
    public Trigger beamIsMadeDebounced;
    public Trigger intakeCurrentAboveFive;
    public Debouncer beambreakDebouncer;
    public DigitalInput fixedBeambreak;

    public NetworkTable sideCarTable;
    public IntegerSubscriber scoringHeight;
    public SideCar sideCar;

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
        beamIsMadeDebounced = new Trigger(()-> !beambreakDebouncer.calculate(beamBrokenUndebounced()));
        intakeCurrentAboveFive = new Trigger(()-> (intake.getSupplyCurrent().getValueAsDouble() > 5));
        fixedBeambreak = new DigitalInput(9);

        sideCar = new SideCar();
        //sideCarTable = inst.getTable("sidecarTable");
        //scoringHeight = sideCarTable.getIntegerTopic("scoringLevel").subscribe(1);
    }

    public boolean beamBrokenUndebounced(){
        return  coralizer.getReverseLimit().getValue() == ReverseLimitValue.ClosedToGround;
    }

    public boolean beamIsBroken(){
        //return fixedBeambreak.get();
        return  !beamIsMadeDebounced.getAsBoolean();
    }
    
    public boolean beamNotBroken(){
    return !beamIsBroken();
    }

    public Command intake(){
        return
        either(
                runIntakeAndCoralizer(()-> 0.6).withTimeout(0.03)
                .andThen(new WaitCommand(0.2-0.03))
                .andThen(runIntakeAndCoralizer(()-> 0.6).until(this::beamIsBroken)
                .andThen(runIntakeAndCoralizer(()-> -0.1).withTimeout(1.1))
                .andThen(runIntakeAndCoralizer(()-> 0.0))
            ),
            (runIntakeAndCoralizer(()-> 0.6).until(this::beamIsBroken)
            .andThen(setFirstBreakStateTrue())
            .andThen(          
                  runIntakeAndCoralizer(()->0.4).until(intakeCurrentBelowThreshold.and(this::beamNotBroken))
                  .andThen(setSafeToRaiseElevator())
                  .andThen(runIntakeAndCoralizer(()-> -0.1).until(this::beamBrokenUndebounced))
                  .andThen(runIntakeAndCoralizer(()-> 0))
            )),
            ()-> sideCar.getScoringLevel().heightInches == 1); //(int)Math.round(scoringHeight.get(2)) == 1).withName("intake");
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

    public Command fastEjectCoral(){
        return runCoralizer(()-> 1).withTimeout(0.3).andThen(setDangerousToRaiseElevator()).andThen(setFirstBreakStateFalse());
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
        SmartDashboard.putBoolean("Beambreak No Debounce", beamBrokenUndebounced());
        SmartDashboard.putBoolean("Has Finished Intaking", safeToRaiseElevator.getAsBoolean());
        SmartDashboard.putBoolean("Has Seen First Break", hasSeenFirstBreak);
        SmartDashboard.putNumber("Coralizer speed", coralizer.getVelocity().getValueAsDouble());
        SmartDashboard.putNumber("Intake current", intake.getSupplyCurrent().getValueAsDouble());
        SmartDashboard.putNumber("Coralizer current", coralizer.getSupplyCurrent().getValueAsDouble());
        SmartDashboard.putBoolean("Intake Current Above Five", intakeCurrentAboveFive.getAsBoolean());
    }
}
