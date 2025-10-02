package frc.robot.subsystems;

import static edu.wpi.first.wpilibj2.command.Commands.either;
import java.util.function.DoubleSupplier;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXSConfiguration;
import com.ctre.phoenix6.hardware.TalonFXS;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorArrangementValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.networktables.IntegerSubscriber;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants;

public class Coralizer extends SubsystemBase {
    // Hardware
    private DigitalInput beamBreak;
    private TalonFXS coralizer;
    private TalonFXS intake;

    // configurations
    private Slot0Configs coralizerConfig;

    // triggers
    public boolean isSafeToRaiseElevator;
    public final Trigger safeToRaiseElevator;
    private boolean hasSeenFirstBreak;
    public final Trigger seenFirstBreak;

    // sidecar & network tables
    public IntegerSubscriber scoringHeight;
    public NetworkTable sideCarTable;
    private final NetworkTableInstance inst = NetworkTableInstance.getDefault();

    // position and velocity
    private double maxVelocity;
    private double encoder;

    public Coralizer() {

        maxVelocity = 0.0;

        sideCarTable = inst.getTable("sidecarTable");

        isSafeToRaiseElevator = true;
        safeToRaiseElevator = new Trigger(() -> isSafeToRaiseElevator);
        hasSeenFirstBreak = true;
        seenFirstBreak = new Trigger(() -> hasSeenFirstBreak);

        coralizerConfig = new Slot0Configs();
        coralizerConfig.kP = .1; // An error of 1 rotation results in 2.4 V output

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

    public Command runIndexerInwardUntilCoralizerEncoderDetectsCoral() {
        return runOnce(() -> {
            encoder = coralizer.getPosition().getValueAsDouble();
            intake.set(0.5);

        }).andThen(run(
            () -> {})
        ).until(
            () -> (coralizer.getPosition().getValueAsDouble() - encoder >= 1.0)

        ).finallyDo( 
            () -> {
                intake.set(0.0);
                coralizer.set(0.0);
                coralizer.setPosition(0.0);
            }
        );
    }

    public Command advanceCoralOntoElevatorUntilCoralizerDetectsPositionChange() {
        return runOnce(() -> {
            coralizer.setPosition(0.0);
            intake.set(0.2);
            coralizer.set(0.2);

        }).andThen(run(() -> {})

        ).until(() -> 
            (coralizer.getPosition().getValueAsDouble() >= 19.5) && (!beamBreak.get())

        ).finallyDo(() -> {
            coralizer.set(0.0);
            intake.set(0.0);
            coralizer.setPosition(0.0);
        });
    }

    public Command rollbackUntilCoralIsNotTooFarOut(){
        return runOnce(()-> {
            coralizer.setPosition(0.0);
            coralizer.set(-0.2);
        }).andThen(run(() -> {})
        ).until(() -> 
            (coralizer.getPosition().getValueAsDouble() <= -2.0) || (beamBreak.get())
        ).finallyDo(() -> {
            coralizer.set(0.0);
            coralizer.setPosition(0.0);
        });
    }

    public Command L1Scoring() {
        return runOnce(
            () -> {
                maxVelocity = 0.0;
                intake.set(0.4);
                coralizer.set(0.4);
            }
        ).andThen(run(
            () -> {
                maxVelocity = Math.max(maxVelocity, intake.getVelocity().getValueAsDouble());
            }
        )).until(
            () -> (intake.getVelocity().getValueAsDouble() < (maxVelocity - 3.0))
        ).finallyDo(
            () -> {
                coralizer.set(0);
                intake.set(0);
                coralizer.setPosition(0);
            }
        );
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

    public Command runIntakeAndCoralizerNoStop(DoubleSupplier speed) {
        return new FunctionalCommand(
                () -> {
                    coralizer.set(speed.getAsDouble());
                    intake.set(speed.getAsDouble());
                },
                () -> {
                },
                interrupted -> {
                    coralizer.set(0);
                    intake.set(0);
                },
                () -> {
                    return false;
                },
                this);
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
                L1Scoring(),
                runIndexerInwardUntilCoralizerEncoderDetectsCoral()
                        .andThen(advanceCoralOntoElevatorUntilCoralizerDetectsPositionChange()).andThen(rollbackUntilCoralIsNotTooFarOut()),
                () -> (int) Math.round(scoringHeight.get(Constants.Coralizer.scoringHeightDefault)) == 1)
                .withName("intake");

    }

    public Command ejectCoral() {
        return runCoralizer(() -> 0.6).withTimeout(0.2).andThen(setDangerousToRaiseElevator())
                .andThen(setFirstBreakStateFalse());
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("max intake velocity", maxVelocity);
        SmartDashboard.putNumber("intake velocity", intake.getVelocity().getValueAsDouble());
        SmartDashboard.putNumber("Coralizer speed", coralizer.getVelocity().getValueAsDouble());
        SmartDashboard.putNumber("Intake current", intake.getSupplyCurrent().getValueAsDouble());
        SmartDashboard.putNumber("Coralizer current", coralizer.getSupplyCurrent().getValueAsDouble());

    }
}
