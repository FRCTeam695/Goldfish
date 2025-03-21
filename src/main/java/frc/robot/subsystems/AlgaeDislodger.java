package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.configs.TalonFXSConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFXS;
import com.ctre.phoenix6.signals.MotorArrangementValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.StaticFeedforwardSignValue;

import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.PubSubOption;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants;

public class AlgaeDislodger extends SubsystemBase{
    private TalonFXS m_talon;
    private TalonFXSConfiguration configFXS;
    private MotionMagicVoltage controlMM;
    public Trigger atSetpoint;

    // 50Hz NetworkTable variables
    // Creates a new field that contains all output variables
    private final NetworkTableInstance inst = NetworkTableInstance.getDefault();
    private final NetworkTable algaeTable = inst.getTable("Algae");
    // Position
    private final DoublePublisher r_masterRotPub = algaeTable.getDoubleTopic("Right master motor rotations").publish(PubSubOption.periodic(0.02));
    private final DoublePublisher rotationsTargetPub = algaeTable.getDoubleTopic("Position Target").publish(PubSubOption.periodic(0.02));
    // Velocity
    private final DoublePublisher velocityPub = algaeTable.getDoubleTopic("Velocity").publish(PubSubOption.periodic(0.02));
    private final DoublePublisher velocityTargetPub = algaeTable.getDoubleTopic("Velocity Target").publish(PubSubOption.periodic(0.02));
    // kS & kG (Feed forward)
    private final DoublePublisher closedLoopPub = algaeTable.getDoubleTopic("Closed Loop Output").publish(PubSubOption.periodic(0.02));
    private final DoublePublisher FFPub = algaeTable.getDoubleTopic("Feed Forward").publish(PubSubOption.periodic(0.02));
    private final DoublePublisher motorVoltagePub = algaeTable.getDoubleTopic("Motor Voltage").publish(PubSubOption.periodic(0.02));

    public AlgaeDislodger() {
        m_talon = new TalonFXS(54); //change ID accordingly
        configFXS = new TalonFXSConfiguration();
        controlMM = new MotionMagicVoltage(0);
        BaseStatusSignal.setUpdateFrequencyForAll(50, m_talon.getPosition(true), m_talon.getVelocity(true),
        m_talon.getClosedLoopReference(true),m_talon.getClosedLoopReferenceSlope(true));
        atSetpoint = new Trigger(
            ()-> Math.abs(controlMM.Position - m_talon.getPosition().getValueAsDouble()) < 1
        );
    
        // Configurations
        configFXS.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        configFXS.CurrentLimits.SupplyCurrentLimitEnable = true;
        configFXS.CurrentLimits.SupplyCurrentLimit = 35;
        configFXS.CurrentLimits.StatorCurrentLimitEnable = true;
        configFXS.CurrentLimits.StatorCurrentLimit = 25;

        configFXS.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
        configFXS.SoftwareLimitSwitch.ForwardSoftLimitThreshold = 40; //rot
        configFXS.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
        configFXS.SoftwareLimitSwitch.ReverseSoftLimitThreshold = -36.64; //rot

        configFXS.Commutation.MotorArrangement = MotorArrangementValue.NEO550_JST;

        
        // PID
        configFXS.Slot0.StaticFeedforwardSign = StaticFeedforwardSignValue.UseClosedLoopSign;
        configFXS.Slot0.kS = 0.25;
        configFXS.Slot0.kV = 0.07;
        configFXS.Slot0.kA = 0.005;
        configFXS.Slot0.kP = 0.5;

        // MM
        configFXS.MotionMagic.MotionMagicCruiseVelocity = 150;
        configFXS.MotionMagic.MotionMagicAcceleration = 1000;

        m_talon.getConfigurator().apply(configFXS);

        m_talon.setPosition(0);
    }

    public Command goToPosition(DoubleSupplier setpoint) {
        return run(() -> m_talon.setControl(controlMM.withPosition(setpoint.getAsDouble())));
    }

    public Command dump(){
        return goToPosition(()-> Constants.Alagizer.dump).until(atSetpoint)
                    .andThen(goToPosition(()-> Constants.Alagizer.dislodgeAngle).until(atSetpoint))
                    .andThen(goToPosition(()-> 0).until(atSetpoint));
    }

    public Command voltageControl(DoubleSupplier setpoint) {
        DutyCycleOut output = new DutyCycleOut(0);
        return run(() -> {
            output.Output = setpoint.getAsDouble();
            m_talon.setControl(output);
        });
    }

    //may need to put this in a command rather than periodic method
    @Override
    public void periodic() {
        // Field variable outputs
        // Position
        r_masterRotPub.set(m_talon.getPosition(true).getValueAsDouble());
        rotationsTargetPub.set(m_talon.getClosedLoopReference(true).getValueAsDouble());
        // Velocity
        velocityPub.set(m_talon.getVelocity(true).getValueAsDouble());
        velocityTargetPub.set(m_talon.getClosedLoopReferenceSlope(true).getValueAsDouble());
        // kS & kG (Feed forward)
        closedLoopPub.set(m_talon.getClosedLoopProportionalOutput(true).getValueAsDouble());
        FFPub.set(m_talon.getClosedLoopFeedForward(true).getValueAsDouble());
        motorVoltagePub.set(m_talon.getMotorVoltage(true).getValueAsDouble());
    }
}