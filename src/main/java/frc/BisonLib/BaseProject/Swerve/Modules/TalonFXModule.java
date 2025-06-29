package frc.BisonLib.BaseProject.Swerve.Modules;

//import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
//import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.ctre.phoenix6.signals.StaticFeedforwardSignValue;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;

import java.util.concurrent.locks.ReentrantReadWriteLock;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;

import frc.robot.Constants;

public class TalonFXModule{
    private final TalonFX driveMotor;
    private final TalonFX turnMotor;

    /*
     * This encoder tells the wheel where it is
     * does not get reset on power cycle, this is what makes it an absolute encoder!
     */
    private final CANcoder absoluteEncoder;

    /*
     * This PID controller is responsible for turning the wheel
     */
    //private final PIDController turnFeedback;

    // private final SimpleMotorFeedforward driveFf;
    // private final PIDController driveController;

    private final PositionVoltage rotationSetter = new PositionVoltage(0.0);
    private final VelocityVoltage velocitySetter = new VelocityVoltage(0.0);

    private SwerveModulePosition latestPosition = new SwerveModulePosition();

    /*
     * The type of module that it is
     */
    public final String kModuleType = "TalonFXModule";

    private double rot_sample;
    private final BaseStatusSignal[] odomSignals = new BaseStatusSignal[4];
    private final ReentrantReadWriteLock odometryLock = new ReentrantReadWriteLock();

    private final BaseStatusSignal drivePositionSignal;
    private final BaseStatusSignal driveVelocitySignal;
    private final BaseStatusSignal rotationSignal;

    /*
     * The module index
     * This follows the quadrants in a cartesian coordinate grid
     * 
     * front right - 0
     * front left - 1
     * back left - 2
     * back right - 3
     */
    public int index;


    public TalonFXModule(int driveMotorId, int turnMotorId, double absoluteEncoderOffset, int TurnCANCoderId, int moduleIndex){
        this.index = moduleIndex;
        driveMotor = new TalonFX(driveMotorId, "drivetrain");
        turnMotor = new TalonFX(turnMotorId, "drivetrain");
        absoluteEncoder = new CANcoder(TurnCANCoderId, "drivetrain");

        configCANcoder(absoluteEncoderOffset);
        configDriveMotor();
        configTurnMotor();

        //Creates the PID controller for turning
        //turningPidController = new PIDController(0.015, 0.0, 0.0);
        //turnFeedback = new PIDController(Constants.Swerve.WHEEL_KP, 0.0, 0);
        //turnFeedback.enableContinuousInput(-180, 180); //Tells the PID controller that 180 and -180 are at the same place

        // driveFf = new SimpleMotorFeedforward(0.011, 0.2);
        // driveController = new PIDController(0.1, 0, 0);
        rot_sample = 0;

        
        drivePositionSignal = driveMotor.getPosition();
        driveVelocitySignal = driveMotor.getVelocity();
        rotationSignal = absoluteEncoder.getAbsolutePosition();

        odomSignals[0] = drivePositionSignal;
        odomSignals[1] = driveVelocitySignal;
        odomSignals[2] = rotationSignal;
    }


    public BaseStatusSignal[] getOdometrySignals(){
        return odomSignals;
    }

    
    /**
     * Configures the drive motor
     */
    public void configDriveMotor(){
        TalonFXConfiguration config = new TalonFXConfiguration();

        // config.Slot0.kP = Constants.Swerve.DRIVE_WHEEL_KP; 
        // config.Slot0.kV = Constants.Swerve.DRIVE_WHEEL_KV; 
        // config.Slot0.kS = Constants.Swerve.DRIVE_WHEEL_KS;

        config.Slot0.kP = 0.2156; 
        config.Slot0.kV = 0.1232; 
        config.Slot0.kS = 0.2;
        config.Slot0.kA = 0.0054112;

        config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        config.CurrentLimits.StatorCurrentLimit = Constants.Swerve.STATOR_CURRENT_LIMIT;
        config.CurrentLimits.SupplyCurrentLimit = Constants.Swerve.SUPPLY_CURRENT_LIMIT;
        config.CurrentLimits.SupplyCurrentLimitEnable = true;
        config.CurrentLimits.StatorCurrentLimitEnable = true;
        config.Audio.AllowMusicDurDisable = true;

        if(Constants.Swerve.MODULE_IS_INVERTED){
            config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        }
        else{
            config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        }

        // Motion Magic (Trapezoid speed control)
        var motionMagicConfigs = config.MotionMagic;
        motionMagicConfigs.MotionMagicCruiseVelocity = 100; 
        motionMagicConfigs.MotionMagicAcceleration = 6.5 * Constants.Swerve.DRIVING_GEAR_RATIO / Constants.Swerve.WHEEL_CIRCUMFERENCE_METERS; 
        //motionMagicConfigs.MotionMagicJerk = 2000;

        driveMotor.getConfigurator().apply(config);
        driveMotor.setPosition(0.0);
    }


    /**
     * Configures the turn motor
     */
    public void configTurnMotor(){
        TalonFXConfiguration config = new TalonFXConfiguration();
        config.ClosedLoopGeneral.ContinuousWrap = true;
        config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        config.CurrentLimits.SupplyCurrentLimit = 30;
        config.CurrentLimits.SupplyCurrentLimitEnable = true;
        config.Audio.AllowMusicDurDisable = true;

        config.Feedback.FeedbackRemoteSensorID = absoluteEncoder.getDeviceID();
        config.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RemoteCANcoder;
        config.Feedback.RotorToSensorRatio = Constants.Swerve.TURNING_GEAR_RATIO;

        if(Constants.Swerve.MODULE_IS_INVERTED){
            config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        }
        else{
            config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        }

        // config.Slot0.kP = Constants.Swerve.TURN_WHEEL_KP;
        // config.Slot0.kD = Constants.Swerve.TURN_WHEEL_KD;
        // config.Slot0.kS = Constants.Swerve.TURN_WHEEL_KS;
        config.Slot0.kP = 43.8;
        config.Slot0.kS = 0.1111;
        config.Slot0.kA = 0.12783;
        config.Slot0.kV = 2.4877;

        config.Slot0.StaticFeedforwardSign = StaticFeedforwardSignValue.UseVelocitySign;
        
        //turnmotor kp = 70, kd = 0, ks = 0.145, ka = 0.12783

        // Motion Magic (Trapezoid speed control)
        var motionMagicConfigs = config.MotionMagic;
        motionMagicConfigs.MotionMagicCruiseVelocity = 100; 
        motionMagicConfigs.MotionMagicAcceleration = 200; 
        //motionMagicConfigs.MotionMagicJerk = 2000;

        turnMotor.getConfigurator().apply(config);
    }

    
    public double getUncachedDrivePosition(){
        return driveMotor.getPosition().getValueAsDouble();
    }
    
    /**
     * Configures the CANcoder,  this involves giving it an offset
     * 
     * @param offset the offset to give the CANcoder
     */
    public void configCANcoder(double offset){
        CANcoderConfiguration cancoderConfigs = new CANcoderConfiguration();
        cancoderConfigs.MagnetSensor.AbsoluteSensorDiscontinuityPoint = 0.5;
        cancoderConfigs.MagnetSensor.MagnetOffset = offset;
        cancoderConfigs.MagnetSensor.SensorDirection = SensorDirectionValue.CounterClockwise_Positive;
        absoluteEncoder.getConfigurator().apply(cancoderConfigs);
    }

    public TalonFX getDriveMotor(){return driveMotor;}
    public TalonFX getTurnMotor(){return turnMotor;}
    public String getModuleType(){return kModuleType;}

    public double getDriveStatorCurrent(){
        return driveMotor.getStatorCurrent().getValueAsDouble();
    }
    
    public void driveWithVoltage(double volts){
        driveMotor.setVoltage(volts);
        SmartDashboard.putNumber("Swerve/Module " + (this.index + 1) + "/Supply Voltage Draw", driveMotor.getSupplyVoltage().getValueAsDouble());
        SmartDashboard.putNumber("Swerve/Module " + (this.index + 1) + "/Voltage Draw", driveMotor.getMotorVoltage().getValueAsDouble());
    }

    public void setTurnMotor(double v){
        turnMotor.set(v);
    }

    public double start_rotation_sample(){
        rot_sample = getRawDrivePosition();
        return rot_sample;
    }

    public double get_change_in_rotation(){
        return Math.abs(getRawDrivePosition() - rot_sample);
    }

    protected double getRawDriveVelocity(){
        return driveVelocitySignal.getValueAsDouble();
    }

    protected double getRawDrivePosition(){
        return drivePositionSignal.getValueAsDouble();
    }

    public double getDriveAcceleration(){
        return driveMotor.getAcceleration().getValueAsDouble() / Constants.Swerve.DRIVING_GEAR_RATIO * Constants.Swerve.WHEEL_CIRCUMFERENCE_METERS;
    }

    public double getTurnVelocity(){
        return turnMotor.getVelocity().getValueAsDouble() / Constants.Swerve.TURNING_GEAR_RATIO * Constants.Swerve.WHEEL_CIRCUMFERENCE_METERS;
    }

    protected double getCANCoderRadians(){
        double angleRad = rotationSignal.getValueAsDouble() * 2 * Math.PI;
        // SmartDashboard.putNumber("Module " + (this.index + 1) +  " Angle Radians", angleRad);
        // SmartDashboard.putNumber("Module " + (this.index + 1) +  " Angle Degrees", Math.toDegrees(angleRad));
        return angleRad;
        //return 0;
    }


    /*
     * Stops the drive and turn motors, drives them to zero velocity
     */
    public void stop(){
        driveMotor.set(0.0);
        turnMotor.set(0.0);
    }


    /**
     * Sets the module to a given state
     * 
     * 2DO - Use feedforward on drive motor
     */
    public void setDesiredState(SwerveModuleState desiredState) {
        Rotation2d latestAngle;

        // this way drive and odometry cant acess the angle in latestPosition at the same time
        odometryLock.readLock().lock();
        try{
            latestAngle = latestPosition.angle;
        }finally{
            odometryLock.readLock().unlock();
        }
        
        // Module optimization (don't turn more than 90 degrees)
        var delta = desiredState.angle.minus(latestAngle);
        SmartDashboard.putNumber("Unoptimized Angle " + this.index + 1, desiredState.angle.getDegrees());
        SmartDashboard.putNumber("Module Angle Delta " + this.index+1, delta.getDegrees());
        if (Math.abs(delta.getDegrees()) > 90.0) {
          desiredState = new SwerveModuleState(
              -desiredState.speedMetersPerSecond, desiredState.angle.rotateBy(Rotation2d.kPi));
        }

        double velocity = Math.cos(Math.abs(desiredState.angle.getRadians() -  latestAngle.getRadians())) * desiredState.speedMetersPerSecond;
        //driveMotor.set(velocity/Constants.Swerve.MAX_SPEED_METERS_PER_SECONDS);
        driveMotor.setControl(
            velocitySetter.withVelocity(velocity * Constants.Swerve.DRIVING_GEAR_RATIO/Constants.Swerve.WHEEL_CIRCUMFERENCE_METERS)
        );
        
        //driveMotor.set(Math.cos(Math.abs(desiredState.angle.getRadians() -  latestAngle.getRadians())) * desiredState.speedMetersPerSecond/Constants.Swerve.MAX_SPEED_METERS_PER_SECONDS);


        turnMotor.setControl(
            //rotationSetter.withPosition(Rotation2d.fromDegrees(175 * Math.signum(desiredState.angle.getDegrees())).getRotations())
            rotationSetter.withPosition(desiredState.angle.getRotations())
        );

        
        SmartDashboard.putNumber("Module " + (this.index+1) + " Desired Velocity", desiredState.speedMetersPerSecond);
        SmartDashboard.putNumber("Module " + (this.index+1) + " Rotation Setpoint Deg", desiredState.angle.getDegrees());
        SmartDashboard.putNumber("Module " + (this.index+1) + " Angular Velocity", turnMotor.getVelocity().getValueAsDouble());
        SmartDashboard.putNumber("Module " + (this.index+1) + " Angular Acceleration", turnMotor.getAcceleration().getValueAsDouble());

        SmartDashboard.putNumber("Module " + (this.index+1) + " Motor Velocity", driveMotor.getVelocity().getValueAsDouble() / Constants.Swerve.DRIVING_GEAR_RATIO * Constants.Swerve.WHEEL_CIRCUMFERENCE_METERS);
        SmartDashboard.putNumber("Module " + (this.index+1) + " PID Desired Velocity", velocity);

        SmartDashboard.putNumber("Module " + (this.index+1) + " Velocity Error", (velocity)-(driveMotor.getVelocity().getValueAsDouble() / Constants.Swerve.DRIVING_GEAR_RATIO * Constants.Swerve.WHEEL_CIRCUMFERENCE_METERS));

        
    }

    public SwerveModulePosition getPosition(){
        latestPosition.distanceMeters = getRawDrivePosition() / Constants.Swerve.DRIVING_GEAR_RATIO * Constants.Swerve.WHEEL_CIRCUMFERENCE_METERS;

        // this way drive and odometry cant acess the angle in latestPosition at the same time
        odometryLock.writeLock().lock();
        try{
            latestPosition.angle = new Rotation2d(getCANCoderRadians());
        }finally{
            odometryLock.writeLock().unlock();
        }
        return latestPosition;
    }

    public SwerveModuleState getState(){
        Rotation2d angle;
        odometryLock.readLock().lock();
        try {
            angle = latestPosition.angle;
        }finally{
            odometryLock.readLock().unlock();
        }
        return new SwerveModuleState(getRawDriveVelocity() / Constants.Swerve.DRIVING_GEAR_RATIO * Constants.Swerve.WHEEL_CIRCUMFERENCE_METERS, angle);
    }

}