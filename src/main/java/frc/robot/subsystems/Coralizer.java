package frc.robot.subsystems;

import static edu.wpi.first.wpilibj2.command.Commands.either;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXSConfiguration;
import com.ctre.phoenix6.controls.ControlRequest;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFXS;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorArrangementValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.networktables.IntegerSubscriber;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
//import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants;

public class Coralizer extends SubsystemBase {
    private DigitalInput beamBreak;
    private TalonFXS coralizer;
    private TalonFXS intake;

    final PositionVoltage m_request;
    private Slot0Configs coralizerConfig;
    private double SupplyCurrent;

    public boolean isSafeToRaiseElevator;
    public static Trigger safeToRaiseElevator;

    private boolean hasSeenFirstBreak = true;
    public Trigger seenFirstBreak = new Trigger(() -> hasSeenFirstBreak);

    public IntegerSubscriber scoringHeight;
    public NetworkTable sideCarTable;
    private final NetworkTableInstance inst = NetworkTableInstance.getDefault();

    private double maxVelocity = 0;

    double encoder;

    public Coralizer() {

        sideCarTable = inst.getTable("sidecarTable");

        isSafeToRaiseElevator = true;
        safeToRaiseElevator = new Trigger(() -> isSafeToRaiseElevator);

        coralizerConfig = new Slot0Configs();
        coralizerConfig.kP = .1; // An error of 1 rotation results in 2.4 V output

        // create a position closed-loop request, voltage output, slot 0 configs
        m_request = new PositionVoltage(0).withSlot(0).withPosition(20.0);

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
        coralizer.getConfigurator().apply(coralizerConfig);

        intake.getConfigurator().apply(config);

        // coralizer.getPosition().setUpdateFrequency(100);
        intake.getSupplyCurrent().setUpdateFrequency(50);
        SupplyCurrent = intake.getSupplyCurrent().getValueAsDouble();

        coralizer.setPosition(0.0);
        coralizer.getSupplyCurrent().setUpdateFrequency(50);

        scoringHeight = sideCarTable.getIntegerTopic("scoringLevel").subscribe(1);

    }

    public Command setDangerousToRaiseElevator() {
        return runOnce(() -> isSafeToRaiseElevator = false);
    }

    public Command setSafeToRaiseElevator() {
        return runOnce(() -> isSafeToRaiseElevator = true);
    }

    public Command pidControlCoralizer() {
        return new FunctionalCommand(
                () -> {

                },
                () -> {
                    SmartDashboard.putNumber("coralizer position", coralizer.getPosition().getValueAsDouble());
                    coralizer.set(0.25);
                },
                (interrupted) -> {
                    // Stop both motors when the command ends or is interrupted
                    intake.set(0.0);
                    coralizer.set(0.0);
                },
                () -> {
                    return false;
                },
                this);
    }

    public Command detectEncoderChange() {
        return new FunctionalCommand(
                () -> {
                    encoder = coralizer.getPosition().getValueAsDouble();
                    intake.set(0.5);
                },
                () -> {
                    SmartDashboard.putNumber("1 E", coralizer.getPosition().getValueAsDouble());
                    // System.out.println("1 E " + coralizer.getPosition().getValueAsDouble());

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
                    intake.set(0.25);
                    coralizer.set(0.25);
                },
                () -> {
                    SmartDashboard.putNumber("2 E", coralizer.getPosition().getValueAsDouble());
                    // System.out.println("2 E " + coralizer.getPosition().getValueAsDouble());
                },
                (interrupted) -> {
                    coralizer.set(0.0);
                    intake.set(0.0);
                    coralizer.setPosition(0.0);
                },
                () -> {
                    if (coralizer.getPosition().getValueAsDouble() >= 20.0)
                        return true;
                    return false;
                },
                this);
    }

    public Command L1Scoring() {
        return new FunctionalCommand(() -> {
        }, 
        () -> {
            maxVelocity = Math.max(maxVelocity, intake.getVelocity().getValueAsDouble());
        }, 
        interrupted -> {
            coralizer.set(0);
            intake.set(0);
            coralizer.setPosition(0);
        }, 
        () -> {
            if (intake.getVelocity().getValueAsDouble() < maxVelocity - 15.0)
                return true;
            return false;
        }, this);
    }

    public Command recordEncoder() {
        return run(() -> {
            SmartDashboard.putNumber("Coralizer encoder", coralizer.getPosition().getValueAsDouble());
            SmartDashboard.putBoolean("beambreak", beamBreak.get());
        });
    }

    public Command runIntakeMotor(DoubleSupplier speed) {
        return run(() -> {
            intake.set(speed.getAsDouble());
        });
    }

    public Command runCoralizer(DoubleSupplier speed) {
        return run(
                () -> {
                    coralizer.set(speed.getAsDouble());
                });
    }

    public Command runIntakeAndCoralizer(DoubleSupplier speed) {
        return run(() -> {
            coralizer.set(speed.getAsDouble());
            intake.set(speed.getAsDouble());
        });
    }

    public Command setFirstBreakStateTrue() {
        return runOnce(() -> {
            hasSeenFirstBreak = true;
        });
    }

    public Command setFirstBreakStateFalse() {
        return runOnce(() -> hasSeenFirstBreak = false);
    }

    public Command intake() {
        return either(
            runIntakeMotor(() -> 0.6),
            detectEncoderChange().andThen(moveCoralizerToElevator()), 
            () -> (int) Math.round(scoringHeight.get(1)) == 1
        ).withName("intake");

    }

    public Command ejectCoral(){
        return runCoralizer(()-> 0.6).withTimeout(0.2).andThen(setDangerousToRaiseElevator()).andThen(setFirstBreakStateFalse());
    }

    @Override
    public void periodic() {

        SmartDashboard.putNumber("intake velocity", intake.getVelocity().getValueAsDouble());
        SmartDashboard.putNumber("Coralizer speed", coralizer.getVelocity().getValueAsDouble());
        SmartDashboard.putNumber("Intake current", intake.getSupplyCurrent().getValueAsDouble());
        SmartDashboard.putNumber("Coralizer current", coralizer.getSupplyCurrent().getValueAsDouble());

    }
}
