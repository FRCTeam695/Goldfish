package frc.robot.subsystems;


import java.util.function.Supplier;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.StaticFeedforwardSignValue;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.IntegerSubscriber;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.PubSubOption;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import static frc.BisonLib.BaseProject.Utilities.*;


public class DuoTalonLift extends SubsystemBase{
    private static final double rotationsPerInch = 52.685/48.5; // ROT/INCH!!!

    // Right master talon
    private TalonFX r_leaderTalon; // RIGHT MOTOR ID 50 IS LEADER
    private MotionMagicVoltage r_leaderRequests;
    private TalonFXConfiguration r_leaderConfigs;

    // Left slave talon
    private TalonFX l_followerTalon; // LEFT MOTOR ID 51 IS FOLLOWER
    private MotionMagicVoltage l_followerRequests;
    private TalonFXConfiguration l_followerConfigs;

    //private VoltageOut r_voltReq; // Only for SysID
    /*
     * Follower motor will not have any PID or MM.
     * It follows all the movement of the leader motor.
     * However it needs soft limits and brake mode in its settings
     * since it's not 'inheriting' the leader's code
     */

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


    public Trigger atSetpoint;
    public Trigger isDeployed;
    public double inchesSetpoint = 0;
    public boolean isRunning = false;

    public TrapezoidProfile heightProfile;

    // Constructor
    public DuoTalonLift (NetworkTableInstance inst) {
        
        // Creates a new field that contains all output variables
        NetworkTable elevatorTable = inst.getTable("Elevator");

        r_masterRotPub = getPubForTopic50Hz(elevatorTable, "Right master motor rotations");
        rotationsTargetPub = getPubForTopic50Hz(elevatorTable, "Position Target");
        // Velocity
        velocityPub = getPubForTopic50Hz(elevatorTable, "Velocity");
        velocityTargetPub = getPubForTopic50Hz(elevatorTable, "Velocity Target");
        // kS & kG (Feed forward)
        closedLoopPub = getPubForTopic50Hz(elevatorTable, "Closed Loop Output");
        FFPub = getPubForTopic50Hz(elevatorTable, "Feed Forward");


        atSetpoint = new Trigger(()-> (Math.abs(r_leaderTalon.getPosition().getValueAsDouble() / rotationsPerInch - inchesSetpoint) < 0.25) && isRunning);
        isDeployed = new Trigger(()-> (Math.abs(r_leaderTalon.getPosition().getValueAsDouble() / rotationsPerInch - Heights.Ground.heightInches) > 0.25));

        // Right leader control (RIGHT MOTOR IS LEADER)
        r_leaderTalon = new TalonFX(50);
        r_leaderRequests = new MotionMagicVoltage(0); // Trapezoid configuration
        r_leaderConfigs = new TalonFXConfiguration(); // All configurations for right master
        r_leaderTalon.setControl(r_leaderRequests.withUpdateFreqHz(50));
        // Inverts the leader motor
        r_leaderConfigs.MotorOutput.Inverted = InvertedValue.Clockwise_Positive; 

        // Left follower control
        l_followerTalon = new TalonFX(51);
        l_followerRequests = new MotionMagicVoltage(0); // Trapezoid configuration
        l_followerConfigs = new TalonFXConfiguration(); // All configurations for left slave
        l_followerTalon.setControl(l_followerRequests.withUpdateFreqHz(50));
        // Follows leader and doesn't invert
        l_followerTalon.setControl(new Follower(r_leaderTalon.getDeviceID(), true));
        
        //r_voltReq = new VoltageOut(0);

        // Limits and modes 
        r_leaderConfigs.MotorOutput.NeutralMode = NeutralModeValue.Brake; // Set leader neutral mode
        l_followerConfigs.MotorOutput.NeutralMode = NeutralModeValue.Brake; // Set follower neutral mode
        // BOTH HAVE CURRENT LIMIT
        r_leaderConfigs.CurrentLimits.SupplyCurrentLimitEnable = true;
        r_leaderConfigs.CurrentLimits.SupplyCurrentLimit = 40; // Amps
        l_followerConfigs.CurrentLimits.SupplyCurrentLimitEnable = true;
        l_followerConfigs.CurrentLimits.SupplyCurrentLimit = 40; // Amps
        // Soft limits (Just leader because follower's encoder is useless)
        r_leaderConfigs.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
        r_leaderConfigs.SoftwareLimitSwitch.ForwardSoftLimitThreshold = Heights.L4.heightInches * rotationsPerInch; // Rotations
        r_leaderConfigs.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
        r_leaderConfigs.SoftwareLimitSwitch.ReverseSoftLimitThreshold = 0;

        // Tuning (Right leader)
        r_leaderConfigs.Slot0.GravityType = GravityTypeValue.Elevator_Static;
        r_leaderConfigs.Slot0.kG = 0.142;
        r_leaderConfigs.Slot0.StaticFeedforwardSign = StaticFeedforwardSignValue.UseClosedLoopSign;
        r_leaderConfigs.Slot0.kS = 0.0442; // Friction 0.07 volt
        // kV and kA pairs with MM
        r_leaderConfigs.Slot0.kV = 0.115;
        r_leaderConfigs.Slot0.kA = 0.0045;
        // kP and kD accounts for errors created by in-match hits
        r_leaderConfigs.Slot0.kP = 3; 
        r_leaderConfigs.Slot0.kD = 0.2; 
        
        // Motion Magic (Right leader)
        int cruiseVel = 90;
        r_leaderConfigs.MotionMagic.MotionMagicCruiseVelocity = cruiseVel; // rot/sec
        int maxAccel = 500;
        r_leaderConfigs.MotionMagic.MotionMagicAcceleration = maxAccel; // rot/sec^2
        r_leaderConfigs.MotionMagic.MotionMagicJerk = 5500; // rot/sec^3

        // Applying both kraken's configs
        r_leaderTalon.getConfigurator().apply(r_leaderConfigs);
        l_followerTalon.getConfigurator().apply(l_followerConfigs);
        r_leaderTalon.setPosition(0); // Reset leader's position
        heightProfile = new TrapezoidProfile(new TrapezoidProfile.Constraints(cruiseVel, maxAccel));
        SmartDashboard.putNumber("Elevator Set Inches", 0);
    }

    public Command goToScoringHeight(Supplier<Heights> scoringLevel){
        return run(()->{
            double newInchesSetpoint = scoringLevel.get().heightInches;

            /*
            int networkTablesHeight = (int)Math.round(scoringHeight.get(2));
            if(networkTablesHeight == 1) newInchesSetpoint = Heights.L1.heightInches;
            else if(networkTablesHeight == 2) newInchesSetpoint = Heights.L2.heightInches;
            else if(networkTablesHeight == 3)  {
                newInchesSetpoint = Heights.L3.heightInches;
                SmartDashboard.putNumber("Elevator Set Inches", newInchesSetpoint);
            }
            else if(networkTablesHeight == 4)  newInchesSetpoint = Heights.L4.heightInches;
            else newInchesSetpoint = Heights.L1.heightInches;
            */
            elevatorSetInches(newInchesSetpoint);
            isRunning = true;
        }).finallyDo(()-> {isRunning = false;});
    }


    public Command configureSetpoint(Heights setpoint){
        return runOnce(()->{
            double newInchesSetpoint = setpoint.heightInches;
            /*
            int networkTablesHeight = (int)Math.round(scoringHeight.get(2));
            if(networkTablesHeight == 1) newInchesSetpoint = Heights.L1.heightInches;
            else if(networkTablesHeight == 2) newInchesSetpoint = Heights.L2.heightInches;
            else if(networkTablesHeight == 3)  newInchesSetpoint = Heights.L3.heightInches;
            else if(networkTablesHeight == 4)  newInchesSetpoint = Heights.L4.heightInches;
            else newInchesSetpoint = Heights.L1.heightInches;
            */
            inchesSetpoint = newInchesSetpoint;
        });
    }

    private void elevatorSetInches(double newInchesSetpoint) {
        inchesSetpoint = newInchesSetpoint;
        double rotationSetpoint = inchesSetpoint * rotationsPerInch;
        r_leaderTalon.setControl(r_leaderRequests.withPosition(rotationSetpoint));
    }

    public double getElevatorTimeToArrival(){
        heightProfile.calculate(0.02, new TrapezoidProfile.State(r_leaderTalon.getPosition().getValueAsDouble(), r_leaderTalon.getVelocity().getValueAsDouble()), new TrapezoidProfile.State(inchesSetpoint * rotationsPerInch, 0));
        double time = heightProfile.timeLeftUntil(inchesSetpoint * rotationsPerInch);
        SmartDashboard.putNumber("Elevator ETA", time);
        return time;
    }

    // Setting elevator leader talon to spin to a certain height
    // a, b, x, y, and right bumper control different set heights (for now)
    public Command setHeightLevel(Heights setpoint) {
        return 
        run(() -> 
        {
            isRunning = true;
            elevatorSetInches(setpoint.heightInches);
        }).finallyDo(()-> {isRunning = false;});
    }

    public Command setHeightLevel (Supplier<Heights> setpoint) {
        return run(() -> 
        {
            elevatorSetInches(setpoint.get().heightInches);
        });
    }

    public Command holdHeight(){
        return run(()->{
            elevatorSetInches(r_leaderTalon.getPosition().getValueAsDouble()/rotationsPerInch);
        });
    }

    public Command slowRaise(double speed){
        return run(()->{
            r_leaderTalon.set(speed);
        });
    }

    // Enum of certain heights
    public enum Heights { // An enum is a class of defined objects
        Ground ("Ground", 0),
        L1 ("L1", 8.5), // *rotationsPerInch
        L2 ("L2", 13.412+0.85),
        L3 ("L3", 29.734),
        L4 ("L4", 56.4214672108);

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
        SmartDashboard.putBoolean("Elevator at Setpoint", atSetpoint.getAsBoolean());
        SmartDashboard.putNumber("Elevator Closed Loop Reference", r_leaderTalon.getClosedLoopReference().getValueAsDouble());
        SmartDashboard.putNumber("Elevator Commaded Position", inchesSetpoint);
        SmartDashboard.putBoolean("Elevator is deployed", isDeployed.getAsBoolean());
        SmartDashboard.putBoolean("Elevator is running", isRunning);
 
        // Field variable outputs
        // Position
        publish(r_masterRotPub, r_leaderTalon.getPosition());
        publish(rotationsTargetPub, r_leaderTalon.getClosedLoopReference());
        // Velocity
        publish(velocityPub, r_leaderTalon.getVelocity());
        publish(velocityTargetPub, r_leaderTalon.getClosedLoopReferenceSlope());
        // kS & kG (Feed forward)
        publish(closedLoopPub, r_leaderTalon.getClosedLoopProportionalOutput());
        publish(FFPub, r_leaderTalon.getClosedLoopFeedForward());
    }
}
