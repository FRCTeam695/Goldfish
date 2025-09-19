package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.configs.TalonFXSConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFXS;
import com.ctre.phoenix6.signals.MotorArrangementValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.StaticFeedforwardSignValue;

import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import static frc.BisonLib.BaseProject.Utilities.*;

import frc.robot.Constants;

public class AlgaeDislodger extends SubsystemBase{
    private TalonFXS m_talon;
    private TalonFXSConfiguration configFXS;
    private MotionMagicVoltage controlMM;
    public final Trigger atSetpoint;

    // 50Hz NetworkTable variables
    // Position
    private final DoublePublisher r_masterRotPub;
    private final DoublePublisher rotationsTargetPub;
    // Velocity
    private final DoublePublisher velocityPub;
    private final DoublePublisher velocityTargetPub;
    // kS & kG (Feed forward)
    private final DoublePublisher closedLoopPub;
    private final DoublePublisher FFPub;
    private final DoublePublisher motorVoltagePub;


    public AlgaeDislodger(NetworkTableInstance inst) {
        m_talon = new TalonFXS(54); //change ID accordingly
        configFXS = new TalonFXSConfiguration();
        controlMM = new MotionMagicVoltage(0);
        BaseStatusSignal.setUpdateFrequencyForAll(50, m_talon.getPosition(true), m_talon.getVelocity(true),
        m_talon.getClosedLoopReference(true),m_talon.getClosedLoopReferenceSlope(true));
        atSetpoint = new Trigger(
            ()-> Math.abs(controlMM.Position - m_talon.getPosition().getValueAsDouble()) < 1
        );

        NetworkTable algaeTable = inst.getTable("Algae");

        r_masterRotPub = getPubForTopic50Hz(algaeTable, "Right master motor rotations");
        rotationsTargetPub = getPubForTopic50Hz(algaeTable, "Position Target");
        // Velocity
        velocityPub = getPubForTopic50Hz(algaeTable, "Velocity");
        velocityTargetPub = getPubForTopic50Hz(algaeTable, "Velocity Target");
        // kS & kG (Feed forward)
        closedLoopPub = getPubForTopic50Hz(algaeTable, "Closed Loop Output");
        FFPub = getPubForTopic50Hz(algaeTable, "Feed Forward");
        motorVoltagePub = getPubForTopic50Hz(algaeTable, "Motor Voltage");
    
        // Configurations
        configFXS.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        configFXS.CurrentLimits.SupplyCurrentLimitEnable = true;
        configFXS.CurrentLimits.SupplyCurrentLimit = 35;
        configFXS.CurrentLimits.StatorCurrentLimitEnable = true;
        configFXS.CurrentLimits.StatorCurrentLimit = 25;

        configFXS.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
        configFXS.SoftwareLimitSwitch.ForwardSoftLimitThreshold = 40; //rot
        configFXS.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
        configFXS.SoftwareLimitSwitch.ReverseSoftLimitThreshold = -48; //rot

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

    public Command voltageControl(DoubleSupplier dutyCycle) {
        return run(runTalonFXSDutyCycle(m_talon, dutyCycle));
    }

    //may need to put this in a command rather than periodic method
    @Override
    public void periodic() {
        // Field variable outputs
        // Position
        publish(r_masterRotPub, m_talon.getPosition());
        publish(rotationsTargetPub, m_talon.getClosedLoopReference());
        // Velocity
        publish(velocityPub, m_talon.getVelocity());
        publish(velocityTargetPub, m_talon.getClosedLoopReferenceSlope());
        // kS & kG (Feed forward)
        publish(closedLoopPub, m_talon.getClosedLoopProportionalOutput());
        publish(FFPub, m_talon.getClosedLoopFeedForward());
        publish(motorVoltagePub, m_talon.getMotorVoltage());
    }
}