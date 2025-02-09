package frc.robot.subsystems;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.StaticFeedforwardSignValue;

import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.PubSubOption;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.units.measure.*;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import static edu.wpi.first.units.Units.Volts;

public class DuoTalonLift extends SubsystemBase{
    private static final double rotationsPerInch = 0; // 1.35833 inch/rot

    // Right master talon
    private TalonFX r_leaderTalon; // RIGHT MOTOR ID 50 IS LEADER
    private MotionMagicVoltage r_leaderRequests;
    private TalonFXConfiguration r_leaderConfigs;

    // Left slave talon
    private TalonFX l_followerTalon; // LEFT MOTOR ID 51 IS FOLLOWER
    private MotionMagicVoltage l_followerRequests;
    private TalonFXConfiguration l_followerConfigs;

    private VoltageOut r_voltReq; // Only for SysID
    /*
     * Follower motor will not have any PID or MM.
     * It follows all the movement of the leader motor.
     * However it needs soft limits and brake mode in its settings
     * since it's not 'inheriting' the leader's code
     */

    // Constructor
    public DuoTalonLift () {
        // Right leader control (RIGHT MOTOR IS LEADER)
        r_leaderTalon = new TalonFX(15);
        r_leaderRequests = new MotionMagicVoltage(0); // Trapezoid configuration
        r_leaderConfigs = new TalonFXConfiguration(); // All configurations for right master
        r_leaderTalon.setControl(r_leaderRequests.withUpdateFreqHz(50));
        // Inverts the leader motor
        r_leaderConfigs.MotorOutput.Inverted = InvertedValue.Clockwise_Positive; 

        // Left follower control
        l_followerTalon = new TalonFX(14);
        l_followerRequests = new MotionMagicVoltage(0); // Trapezoid configuration
        l_followerConfigs = new TalonFXConfiguration(); // All configurations for left slave
        l_followerTalon.setControl(l_followerRequests.withUpdateFreqHz(50));
        // Follows leader and doesn't invert
        l_followerTalon.setControl(new Follower(r_leaderTalon.getDeviceID(), true));
        
        r_voltReq = new VoltageOut(0);

        // Limits and modes 
        r_leaderConfigs.MotorOutput.NeutralMode = NeutralModeValue.Brake; // Set leader neutral mode
        l_followerConfigs.MotorOutput.NeutralMode = NeutralModeValue.Brake; // Set follower neutral mode
        // BOTH HAVE CURRENT LIMIT
        r_leaderConfigs.CurrentLimits.SupplyCurrentLimitEnable = true;
        r_leaderConfigs.CurrentLimits.SupplyCurrentLimit = 20; // Amps
        l_followerConfigs.CurrentLimits.SupplyCurrentLimitEnable = true;
        l_followerConfigs.CurrentLimits.SupplyCurrentLimit = 20; // Amps
        // Soft limits (Just leader because follower's encoder is useless)
        r_leaderConfigs.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
        r_leaderConfigs.SoftwareLimitSwitch.ForwardSoftLimitThreshold = 50; // Rotations
        r_leaderConfigs.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
        r_leaderConfigs.SoftwareLimitSwitch.ReverseSoftLimitThreshold = 0;

        // Tuning (Right leader)
        r_leaderConfigs.Slot0.GravityType = GravityTypeValue.Elevator_Static;
        r_leaderConfigs.Slot0.kG = 0;
        r_leaderConfigs.Slot0.StaticFeedforwardSign = StaticFeedforwardSignValue.UseClosedLoopSign;
        r_leaderConfigs.Slot0.kS = 0; // Friction 0.07 volt
        // kV and kA pairs with MM
        r_leaderConfigs.Slot0.kV = 0.1;
        r_leaderConfigs.Slot0.kA = 0;
        // kP and kD accounts for errors created by in-match hits
        r_leaderConfigs.Slot0.kP = 0; 
        r_leaderConfigs.Slot0.kD = 0; 

        // Motion Magic (Right leader)
        r_leaderConfigs.MotionMagic.MotionMagicCruiseVelocity = 100; // rot/sec
        r_leaderConfigs.MotionMagic.MotionMagicAcceleration = 150; // rot/sec^2
        r_leaderConfigs.MotionMagic.MotionMagicJerk = 2000; // rot/sec^3

        // Applying both kraken's configs
        r_leaderTalon.getConfigurator().apply(r_leaderConfigs);
        l_followerTalon.getConfigurator().apply(l_followerConfigs);
        r_leaderTalon.setPosition(0); // Reset leader's position
    }

    // Setting elevator leader talon to spin to a certain height
    // a, b, x, y, and right bumper control different set heights (for now)
    public Command setHeightLevel(Heights setpoint) {
        return runOnce(() -> 
        {
            r_leaderTalon.setControl(r_leaderRequests.withPosition(setpoint.heightInches*rotationsPerInch));
        });
    }

    // SysID
    private SysIdRoutine m_sysIdRoutine = new SysIdRoutine(
        new SysIdRoutine.Config(
            null, // Default ramp rate (1V/s)
            Volts.of(4), // Reduce dynamic step voltage to 4 to prevent brownout
            null, // Default timeout (10s)

            (state) -> SignalLogger.writeString("State", state.toString())
        ), 
        new SysIdRoutine.Mechanism(
            (volts) -> r_leaderTalon.setControl(r_voltReq.withOutput(volts.in(Volts))),
            null, // Left null when using a signal logger
            this
        )
    );
    // SysID (Quasistatic)
    public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
        return m_sysIdRoutine.quasistatic(direction);
    }
    // SysID (Dynamic)
    public Command sysIdDynamic(SysIdRoutine.Direction direction) {
        return m_sysIdRoutine.dynamic(direction);
    }

    // Enum of certain heights
    public enum Heights { // An enum is a class of defined objects
        Ground ("Ground", 0),
        L1 ("L1", 0),
        L2 ("L2", 0),
        L3 ("L3", 0),
        L4 ("L4", 0);

        String level;
        double heightInches;

        // Constructor
        Heights(String level, double heightInches) {
            this.level = level;
            this.heightInches = heightInches;
        }
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Motor Voltage output", r_leaderTalon.getMotorVoltage().getValueAsDouble());
        SmartDashboard.putNumber("Supply Voltage output", l_followerTalon.getSupplyVoltage().getValueAsDouble());
        SmartDashboard.putNumber("Position", r_leaderTalon.getPosition().getValueAsDouble());
    }
}

/*
// 50Hz NetworkTable variables
    // Creates a new field that contains all output variables
    private final NetworkTableInstance inst = NetworkTableInstance.getDefault();
    private final NetworkTable elevatorTable = inst.getTable("Elevator");
 * // Position
    private final DoublePublisher r_masterRotPub = elevatorTable.getDoubleTopic("Right master motor rotations").publish(PubSubOption.periodic(0.02));
    private final DoublePublisher l_slaveRotPub = elevatorTable.getDoubleTopic("Left slave motor rotations").publish(PubSubOption.periodic(0.02));
    private final DoublePublisher rotationsTargetPub = elevatorTable.getDoubleTopic("Position Target").publish(PubSubOption.periodic(0.02));
    // Velocity
    private final DoublePublisher velocityPub = elevatorTable.getDoubleTopic("Velocity").publish(PubSubOption.periodic(0.02));
    private final DoublePublisher velocityTargetPub = elevatorTable.getDoubleTopic("Velocity Target").publish(PubSubOption.periodic(0.02));
    // kS & kG (Feed forward)
    private final DoublePublisher closedLoopPub = elevatorTable.getDoubleTopic("Closed Loop Output").publish(PubSubOption.periodic(0.02));
    private final DoublePublisher FFPub = elevatorTable.getDoubleTopic("Feed Forward").publish(PubSubOption.periodic(0.02));
 */
