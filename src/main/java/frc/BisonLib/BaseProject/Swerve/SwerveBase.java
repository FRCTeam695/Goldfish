package frc.BisonLib.BaseProject.Swerve;

import static edu.wpi.first.units.Units.Volts;
import static edu.wpi.first.wpilibj2.command.Commands.deadline;
import static edu.wpi.first.wpilibj2.command.Commands.waitSeconds;

import java.util.concurrent.locks.ReentrantReadWriteLock;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.Orchestra;
import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.controls.VoltageOut;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.ModuleConfig;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.util.DriveFeedforwards;
import com.pathplanner.lib.util.swerve.SwerveSetpoint;
import com.pathplanner.lib.util.swerve.SwerveSetpointGenerator;
import com.studica.frc.AHRS;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.BisonLib.BaseProject.LimelightHelpers;
import frc.BisonLib.BaseProject.Swerve.Modules.TalonFXModule;
import frc.robot.Constants;

public class SwerveBase extends SubsystemBase {

    protected TalonFXModule[] modules;

    protected final Field2d m_field = new Field2d();
    private final SwerveDrivePoseEstimator odometry;

    // NEVER DIRECTLY CALL ANY GYRO METHODS, ALWAYS USE THE SYNCHRONIZED GYRO LOCK!!
    private final AHRS gyro = new AHRS(AHRS.NavXComType.kMXP_SPI, Constants.Swerve.ODOMETRY_UPDATE_RATE_HZ_INTEGER);


    // private final BuiltInAccelerometer rioAccelerometer = new BuiltInAccelerometer();
    // private final LinearFilter xAccelFilter = LinearFilter.movingAverage(5);
    // private final LinearFilter yAccelFilter = LinearFilter.movingAverage(5);
    private final PIDController thetaController = new PIDController(Constants.Swerve.ROBOT_ROTATION_KP, 0, 0);
    private final BaseStatusSignal[] allOdomSignals;

    protected double max_accel = 0;
    protected double speed = 0;
    protected boolean rotatedToSetpoint = false;

    // used for wheel characterization
    protected double initialGyroAngle = 0;
    protected double[] initialPositions = new double[4];

    protected double lastTime = Timer.getFPGATimestamp();
    private LinearFilter lowpass = LinearFilter.movingAverage(50);
    protected double currentTime = 0;
    protected double totalLoopTime = 0;
    protected double inc = 0;
    protected double avgLoopTIme = 0;
    protected double failedOdometryUpdates = 0;
    protected double successfulOdometryUpdates = 0;

    public final Trigger atRotationSetpoint = new Trigger(()-> robotRotationAtSetpoint());
    public PPHolonomicDriveController pathplannerController =  new PPHolonomicDriveController( // HolonomicPathFollowerConfig, this should likely live in your Constants class
                                                                new PIDConstants(6, 0.0, 0.1), // Translation PID constants (JPK was 6,0,0)
                                                                new PIDConstants(5, 0.0, 0.0) // Rotation PID constants (JPK was 2)
                                                                );



    // this is a lock to make sure nobody acesses our pose while odometry is updating it
    private final ReentrantReadWriteLock odometryLock = new ReentrantReadWriteLock();

    // this is a lock to make sure nobody acesses our gyro while odometry is updating it
    private final Object gyroLock = new Object();

    private Pose2d currentRobotPose = new Pose2d();
    private SwerveModulePosition[] currentModulePositions = new SwerveModulePosition[4];
    private SwerveModuleState[] currentModuleStates = new SwerveModuleState[4];

    protected String[] camNames;
    private final SwerveSetpointGenerator setpointGenerator;
    private SwerveSetpoint previousSetpoint;
 
    SlewRateLimiter omegaFilter = new SlewRateLimiter(Math.toRadians(1074.5588535));
    SlewRateLimiter xFilter = new SlewRateLimiter(Constants.Swerve.MAX_ACCELERATION_METERS_PER_SECOND_SQ);
    SlewRateLimiter yFilter = new SlewRateLimiter(Constants.Swerve.MAX_ACCELERATION_METERS_PER_SECOND_SQ);
    SlewRateLimiter accelFilter = new SlewRateLimiter(Constants.Swerve.MAX_ACCELERATION_METERS_PER_SECOND_SQ);
    //private Pigeon2 pigeon = new Pigeon2(8);

    private VoltageOut m_voltReq;

    /**
     * Does all da constructing
     * 
     * @param cameras An array of cameras used for pose estinmation
     * @param moduleTypes The type of swerve module on the swerve drive
     */
    public SwerveBase(String[] camNames, TalonFXModule[] modules) {
        //pigeon.setYaw(0);
        // 4 modules * 3 signals per module
        allOdomSignals = new BaseStatusSignal[(4 * 3)];
        for(int i = 0; i < modules.length; ++i){
            var signals = modules[i].getOdometrySignals();
            allOdomSignals[i*3 + 0] = signals[0]; // drive position
            allOdomSignals[i*3 + 1] = signals[1]; // drive velocity
            allOdomSignals[i*3 + 2] = signals[2]; // module rotation (cancoder)
        }

        this.camNames = camNames;

        // Holds all the modules
        this.modules = modules;

        RobotConfig config;
        try{
            config = RobotConfig.fromGUISettings();
        } catch (Exception e) {
        // Handle exception as needed
            e.printStackTrace();
            config = new RobotConfig(0, 0, new ModuleConfig(0.051, 5.3, 1.15, DCMotor.getFalcon500(1), 40, 4), Constants.Swerve.FRONT_LEFT_TRANSLATION, Constants.Swerve.FRONT_RIGHT_TRANSLATION, Constants.Swerve.BACK_LEFT_TRANSLATION, Constants.Swerve.BACK_RIGHT_TRANSLATION);
        }

        initAutoBuilder(config);


        /*
        * Sets the gyro at the beginning of the match and 
        * also sets each module state to present
        */

        //maybe have this keep trying to reset the gyro if it fails once
        new Thread(() -> {
            try {
                Thread.sleep(1000);
                setGyro(0);
            } catch (Exception e) {
                setGyro(0);
            }
        }).start();


        thetaController.enableContinuousInput(-180, 180);


        odometryLock.writeLock().lock();
        Rotation2d gyroHeading = getGyroHeading();
        currentModulePositions = getModulePositions();
        try{
            odometry = new SwerveDrivePoseEstimator
            (
                Constants.Swerve.kDriveKinematics, 
                gyroHeading,
                currentModulePositions,
                new Pose2d()
            );
        }finally{
            odometryLock.writeLock().unlock();
        }


        setpointGenerator = new SwerveSetpointGenerator(
            config, // The robot configuration. This is the same config used for generating trajectories and running path following commands.
            Units.rotationsToRadians(Constants.Swerve.MAX_WHEEL_ROTATIONAL_SPEED) // The max rotation velocity of a swerve module in radians per second. This should probably be stored in your Constants file
        );

        
        ChassisSpeeds speeds = new ChassisSpeeds();
        SwerveModuleState[] states = new SwerveModuleState[] {new SwerveModuleState(), new SwerveModuleState(), new SwerveModuleState(), new SwerveModuleState()};
        previousSetpoint = new SwerveSetpoint(speeds, states, DriveFeedforwards.zeros(config.numModules));


        SmartDashboard.putData("field", m_field);
        SmartDashboard.putData("Robot angle PID controller", thetaController);

        m_voltReq = new VoltageOut(0.0); 
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
            (volts) -> {
                modules[0].getDriveMotor().setControl(m_voltReq.withOutput(volts.in(Volts)));
                modules[1].getDriveMotor().setControl(m_voltReq.withOutput(volts.in(Volts)));
                modules[2].getDriveMotor().setControl(m_voltReq.withOutput(volts.in(Volts)));
                modules[3].getDriveMotor().setControl(m_voltReq.withOutput(volts.in(Volts)));
            },
            null, // Left null when using a signal logger
            this
        )
    );

    public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
        return m_sysIdRoutine.quasistatic(direction);
    }
     
    public Command sysIdDynamic(SysIdRoutine.Direction direction) {
        return m_sysIdRoutine.dynamic(direction);
    }


    public void playSong(){
        if(modules[0].getModuleType().equals("TalonFXModule")){
            new Thread(() -> {
                try {
                        Orchestra orchestra = new Orchestra();
                        for(var module : this.modules){
                            orchestra.addInstrument(module.getDriveMotor());
                            orchestra.addInstrument(module.getTurnMotor());
                        }

                        var status  = orchestra.loadMusic("EmpireStrikesBack.chrp");
                        if(status.isOK()){
                            double startTime = Timer.getFPGATimestamp();
                            orchestra.play();
                            while((Timer.getFPGATimestamp() - startTime) < 10){}
                            orchestra.stop();
                        }
                        orchestra.close();
                } catch (Exception e) {
                }
            }).start();
        }
    }


    /*
     * Initializes autobuilder,
     * this is used by pathplanner
     */
    public void initAutoBuilder(RobotConfig config){

        // Configure the AutoBuilder last
        AutoBuilder.configure(
                this::getSavedPose, // Robot pose supplier
                this::resetOdometry, // Method to reset odometry (will be called if your auto has a starting pose)
                this::getLatestChassisSpeed, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
                (speeds, feedforwards) -> driveRobotRelative(speeds, false), // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds
                pathplannerController,
                config,
                this::isRedAlliance,
            this // Reference to this subsystem to set requirements
        );
    }


    /*
     * isRedAlliance returns true if we are red alliance and returns false if we are blue alliance
     */
    public boolean isRedAlliance(){
        var alliance = DriverStation.getAlliance();
              if (alliance.isPresent()) {
                boolean temp = (alliance.get() == DriverStation.Alliance.Red) ? true : false;
                //SmartDasboard.putBoolean("Alliance", temp);
                return temp;
              }
        //SmartDasboard.putBoolean("Alliance", false);
        return false;
    }


    /*
     * Sets all the swerve modules to the states we want them to be in (velocity + angle)
     */
    public void setModules(SwerveModuleState[] desiredStates) {
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, Constants.Swerve.MAX_SPEED_METERS_PER_SECONDS);

        for(var module : modules){
            //SmartDashboard.putString("Swerve/Module State " + module.index, desiredStates[module.index].toString());
            module.setDesiredState(desiredStates[module.index]);
        }
        
    }


    /**
     * setGyro sets the gyro to a given angle,
     * @param degrees the degree value that the gyro should be set to
     */
    public void setGyro(double degrees){
        //pigeon.setYaw(0);
        synchronized(gyroLock){
            gyro.reset();
            gyro.setAngleAdjustment(-degrees);
        }
    }


    /**
     * checks if the robot is facing the direction of the rotation override
     * 
     * @return if the robot is facing the rotation override angle
     */
    public boolean robotRotationAtSetpoint(){
        return rotatedToSetpoint;
    }


    /**
     * Finds a new angular speed based on rotation override
     * 
     * @param originalSpeeds The original chassis speeds of the robot as inputted by the driver
     * 
     * @return The new rotation component as calculated by the rotation override
     */
    public double getAngularComponentFromRotationOverride(double wantedAngle){
        double currentRotation = getSavedPose().getRotation().getDegrees();
        double pidOutput = thetaController.calculate(currentRotation, wantedAngle);

        rotatedToSetpoint = Math.abs(currentRotation - wantedAngle) < 1;
        return MathUtil.clamp(pidOutput, -1, 1) * Constants.Swerve.MAX_ANGULAR_SPEED_RAD_PER_SECOND;
    }


    /**
     * This method should not be touched by anything except odometry, 
     * all angles should be pulled from odometry and not directly from the gyro.
     * This is done to ensure that there is no thread contention for this method.
     * 
     * @returns the angle the gyro is facing expressed as a Rotation2d
     */
    private Rotation2d getGyroHeading() {
        //0.99622314806
        synchronized (gyroLock){
            return new Rotation2d(-Math.toRadians(Math.IEEEremainder(gyro.getAngle()/0.99622314806, 360)));
        }
    }

    // protected double getGyroRate() {
    //     return pigeon.getAngularVelocityZWorld().getValueAsDouble();
    // }


    /**
     * calculates the distance from the current robot pose to the supplied translation,
     * can be potentially used for figuring out if the robot is in some specific zone
     * 
     * @param other The translation to find the distance to
     * @return The distance from the robot pose to the other supplied translation
     */
    public double getDistanceToTranslation(Translation2d other){
        return getSavedPose().getTranslation().getDistance(other);
    }


    /*
     * Returns the current chassis speeds of the robot, 
     * used with pathplanner
     */
    public ChassisSpeeds getLatestChassisSpeed(){
        ChassisSpeeds speeds;
        odometryLock.readLock().lock();
        try{
            speeds = Constants.Swerve.kDriveKinematics.toChassisSpeeds(currentModuleStates);
        }finally{
            odometryLock.readLock().unlock();
        }
        return speeds;
    }


    public SwerveModuleState[] getLatestModuleStates(){
        SwerveModuleState[] states;
        odometryLock.readLock().lock();
        try{
            states = currentModuleStates;
        }finally{
            odometryLock.readLock().unlock();
        }
        return states;
    }
    
    

    public double getLatestSpeed(){
        return speed;
    }


    /**
     * @returns an array containing the position of each swerve module (check SwerveModule.java for further details)
     */
    public SwerveModulePosition[] getModulePositions() {

        SwerveModulePosition[] modulePositions = new SwerveModulePosition[4];

        for(var module : modules){
            modulePositions[module.index] = module.getPosition();
        }

        return modulePositions;
    }


    /**
     * @return an array containing the state (velocity + rotation) of each swerve module
     */
    public SwerveModuleState[] getModuleStates(){
        SwerveModuleState[] states = new SwerveModuleState[4];

        for(var module : modules){
            states[module.index] = module.getState();
        }

        return states;
    }


    /**
     * Gets the acceleration of each swerve module, 
     * keep in mind these can be slightly inaccurate because of wheel slippage
     * 
     * @return a double array that contains the acceleration of each swerve module
     */
    public double[] getModuleAccelerations(){
        double[] accelerations = new double[4];

        for(var module : modules){
            accelerations[module.index] = module.getDriveAcceleration();
        }

        return accelerations;
    }


    public double[] getRawDrivePositions(){
        double[] positions = new double[4];

        for(var module : modules) {
            positions[module.index] = module.getUncachedDrivePosition();
        }

        return positions;
    }


    /**
     * Manually resets the odometry to a given pose
     * Also resets the gyro
     * Pretty much only used at the start of auton
     * 
     * @param pose The pose to set the robot pose to
     */
    public void resetOdometry(Pose2d pose) {
        setGyro(pose.getRotation().getDegrees());
        Rotation2d gyroHeading = getGyroHeading();
        odometryLock.writeLock().lock();
        try{
            odometry.resetPosition(
                gyroHeading,
                currentModulePositions,
                pose);
        }finally{
            odometryLock.writeLock().unlock();
        }
    }


    /**
     * resetGyro sets the gyro to "facing away from the driver station"
     * 
     * @return A command that "zeroes" our gyro
     */
    public Command resetGyro() {
        return resetGyro(180);
    }
    

    public Command resetGyro(double angle){
        return runOnce(
            ()-> 
                {
                    if(isRedAlliance()){
                        odometryLock.writeLock().lock();
                        try{
                            resetOdometry(new Pose2d(currentRobotPose.getX(), currentRobotPose.getY(), Rotation2d.fromDegrees(angle)));
                        }finally{
                            odometryLock.writeLock().unlock();
                        }
                    }
                    else {
                        odometryLock.writeLock().lock();
                        try{
                            resetOdometry(new Pose2d(currentRobotPose.getX(), currentRobotPose.getY(), Rotation2d.fromDegrees(angle+180)));
                        }finally{
                            odometryLock.writeLock().unlock();
                        }
                    }
                    //LL RESET
                    new Thread(() -> {
                        try {
                            for(String cam: camNames){
                                LimelightHelpers.SetIMUMode(cam, 1);
                                double startTime = Timer.getFPGATimestamp();
                                while( Timer.getFPGATimestamp() < startTime + .1){
                                    LimelightHelpers.SetRobotOrientation(cam, getSavedPose().getRotation().getDegrees(), 0, 0, 0, 0, 0);
                                }
                                LimelightHelpers.SetIMUMode(cam, 3);
                            }
                        } catch (Exception e) {
                        }
                    }).start();
                }
        ).ignoringDisable(true);
    }

    /*
     * Calculates the circumference of the wheel by turning in place slowly
     * 
     * COMMENT OUT DRIVEBASE AND ODOMETRY CODE BEFORE RUNNING THIS
     */
    public Command runWheelCharacterization(){

        // total distance each module should travel for one rotation
        // circumferenc = pi * d
        double one_rotation_distance = Constants.Swerve.WHEEL_BASE_METERS * Math.PI * Math.sqrt(2);

        return runOnce(()-> {

            //get each module in positions
            for(var mod : modules){
                    mod.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45 + mod.index * 90)));
            }

        })
        .andThen(
         waitSeconds(0.5)
        )
        .andThen(runOnce(
          ()-> {
            initialPositions = getRawDrivePositions();
            initialGyroAngle = gyro.getAngle();
          }
        ))
        .andThen(
            deadline(
                new WaitCommand(6)
                ,
                run(()-> {
                    // closed loop control to turn in place one rotation
                    for(var mod : modules){
                        mod.setDesiredState(new SwerveModuleState(0.3, Rotation2d.fromDegrees(45 + mod.index * 90)));
                    }
                })
                )
        )
        .andThen(
            waitSeconds(1)
        )
        .andThen(
            runOnce(()-> {
                double[] currentPositions = getRawDrivePositions();
                double avg_calculated_wheel_circumference = 0;
                double actual_distance_traveled = one_rotation_distance * Math.abs(gyro.getAngle()-initialGyroAngle)/360;
                for(var mod : modules){
                    //original_circumference/new_circumference = calculated_distance/actual_distance
                    // actual_distance * original_circumference = new_circumference * calculated_distance
                    // new_circumference = actual_distance * original_circumference / (calculated_distance)
                    double new_circumference = actual_distance_traveled * Constants.Swerve.WHEEL_CIRCUMFERENCE_METERS / (initialPositions[mod.index] - currentPositions[mod.index]);
                    avg_calculated_wheel_circumference += new_circumference;
                    SmartDashboard.putNumber("initial distance " + mod.index, initialPositions[mod.index]);
                    SmartDashboard.putNumber("current distance " + mod.index, currentPositions[mod.index]);
                    SmartDashboard.putNumber(mod.index + "calculated wheel circumference", new_circumference);
                }
                avg_calculated_wheel_circumference /= 4;
                SmartDashboard.putNumber("Average Calculated Wheel Circumference", avg_calculated_wheel_circumference);
            })
        )
        ;
    }
    

    /*
     * Stops all of the swerve modules
     */
    public void stopModules() {
        for(var module : modules){
            module.stop();
        }
    }


    /**
     * Continuously rotates robot to the specified angle while maintaining normal driver control of the robot
     * 
     * @param angleDegrees The angle in degrees that the robot should turn to, 
     *                     this is a double supplier so you can continously pass different values
     * 
     * @return Returns a functional command that will rotate the robot to a specified angle, 
     *         when interrupted, will return driver control to robot rotation
     */
    public Command rotateToAngle(DoubleSupplier angleDegrees, Supplier<ChassisSpeeds> speedSupplier){
        return run
        (
            /* EXCECUTE */
            ()-> {
                    ChassisSpeeds speeds = speedSupplier.get();
                    speeds.omegaRadiansPerSecond = getAngularComponentFromRotationOverride(angleDegrees.getAsDouble());
                    drive(speeds, true);
                 }
        );
    }


    public Command driveToPose(Pose2d targetPose, double endVelocity){
        PathConstraints constraints = new PathConstraints(
            Constants.Swerve.MAX_SPEED_METERS_PER_SECONDS, Constants.Swerve.MAX_ACCELERATION_METERS_PER_SECOND_SQ,
            Constants.Swerve.MAX_ANGULAR_SPEED_RAD_PER_SECOND, Constants.Swerve.MAX_ACCELERATION_RADIANS_PER_SECOND_SQUARED
        );

        Command pathfindingCommand = AutoBuilder.pathfindToPose(
            targetPose,
            constraints,
            endVelocity // Goal end velocity in meters/sec
        );

        return pathfindingCommand;
    }

    /**
     * Drives swerve given chassis speeds robot relative
     * 
     * @param chassisSpeeds The chassis speeds the robot should travel at
     */
    private void driveRobotRelative(ChassisSpeeds chassisSpeeds, boolean useSetpointGenerator) {
        if(!useSetpointGenerator){
            var tmpStates = Constants.Swerve.kDriveKinematics.toSwerveModuleStates(chassisSpeeds);
            SwerveDriveKinematics.desaturateWheelSpeeds(tmpStates, Constants.Swerve.MAX_SPEED_METERS_PER_SECONDS);
            var speeds = Constants.Swerve.kDriveKinematics.toChassisSpeeds(tmpStates);
            // discretizes the chassis speeds (acccounts for robot skew)
            chassisSpeeds = ChassisSpeeds.discretize(speeds, Constants.Swerve.DISCRETIZE_TIMESTAMP);

            //SmartDashboard.putString("Swerve/Commanded Chassis Speeds", chassisSpeeds.toString());
            // convert chassis speeds to module states
            SwerveModuleState[] moduleStates = Constants.Swerve.kDriveKinematics.toSwerveModuleStates(chassisSpeeds);

            // set the modules to their desired speeds
            setModules(moduleStates);
        }
        else{
            previousSetpoint = setpointGenerator.generateSetpoint(
                previousSetpoint, // The previous setpoint
                chassisSpeeds, // The desired target speeds
                Constants.Swerve.DISCRETIZE_TIMESTAMP // The loop time of the robot code, in seconds
            );

            //set the modules to their desired speeds
            setModules(previousSetpoint.moduleStates());
        }
    }


    /**
     * THIS DOESN'T WORK!!!!!!
     * but it would be really cool if it did, maybe if someone has time then they fix it!?
     * 
     * @param chassisSpeeds The chassis speeds the robot should drive with
     */
    public void driveSwerveFromChassisSpeedsCustomCenterOfRotation(ChassisSpeeds chassisSpeeds){

        // discretizes the chassis speeds (acccounts for robot skew)
        chassisSpeeds = ChassisSpeeds.discretize(chassisSpeeds, Constants.Swerve.DISCRETIZE_TIMESTAMP);
        //SmartDasboard.putNumber("Swerve/chassis x", chassisSpeeds.vxMetersPerSecond);
        //SmartDasboard.putNumber("Swerve/chassis y", chassisSpeeds.vyMetersPerSecond);
        //SmartDasboard.putNumber("Swerve/chassis omega", chassisSpeeds.omegaRadiansPerSecond);


        // I derivated whole thing using polar coordinates but the Translation2d turns it back into standard x, y coordinates
        // Here is my work, I didn't write this in a way that I mean't to be easy to be understood by others but make of it what you will
        // it also doesn't work at all lol
        // https://i.imgur.com/jL4c0yS.jpeg
        double radius = Math.sqrt(2) * Constants.Swerve.WHEEL_BASE_METERS/2.0;
        double offset;

        // Depending on which direction we want to rotate we choose a different center of rotation
        if(chassisSpeeds.omegaRadiansPerSecond > 0){
            offset = Math.PI/4;
        }
        else{
            offset = -Math.PI/4;
        }
        SwerveModuleState[] moduleStates = 
                Constants.Swerve.kDriveKinematics.toSwerveModuleStates(
                    chassisSpeeds,
                    new Translation2d(
                        radius,
                        new Rotation2d(Math.atan(chassisSpeeds.vxMetersPerSecond / chassisSpeeds.vyMetersPerSecond) + offset)
                    )
                );

        // set the modules to their desired speeds
        setModules(moduleStates);
    }

    /*
     * Drives the robot in teleop, we don't want it fighting the auton swerve commands
     */
    public void teleopDefaultCommand(Supplier<ChassisSpeeds> speedsSupplier, boolean fieldOriented){
        drive(speedsSupplier.get(), true);
    }
    
    /**
     * Drives swerve given chassis speeds
     * Should be called every loop
     * 
     * @param speeds the commanded chassis speeds from the joysticks
     * @param fieldOriented A boolean that specifies if the robot should be driven in fieldOriented mode or not
     */
    public void drive(ChassisSpeeds speeds, boolean fieldOriented){
        if(DriverStation.isAutonomous()){
            return;
        }

        speeds.vxMetersPerSecond = xFilter.calculate(speeds.vxMetersPerSecond);
        speeds.vyMetersPerSecond = yFilter.calculate(speeds.vyMetersPerSecond);
        speeds.omegaRadiansPerSecond = omegaFilter.calculate(speeds.omegaRadiansPerSecond);
        //speeds = applyAccelerationLimit(speeds);

        if (fieldOriented) {
            speeds = ChassisSpeeds.fromFieldRelativeSpeeds(speeds, getSavedPose().getRotation());
        }

        this.driveRobotRelative(speeds, false);
    }

    public ChassisSpeeds applyAccelerationLimit(ChassisSpeeds desiredSpeeds){
        // the current measured robot speeds
        ChassisSpeeds current = getLatestChassisSpeed();

        // acceleration in the x and y direction
        double ax = (desiredSpeeds.vxMetersPerSecond - current.vxMetersPerSecond)/0.02;
        double ay = (desiredSpeeds.vyMetersPerSecond - current.vyMetersPerSecond)/0.02;

        // magnitude of the acceleration
        double magnitude = Math.hypot(ax, ay);

        // rate limited magnitude
        double newMagnitude = accelFilter.calculate(magnitude);

        if(magnitude != 0){
            // scale down acceleration in the x and y direction
            double scale = newMagnitude/magnitude;

            ax *= scale;
            ay *= scale;
        }

        // 0.02 is the loop time
        current.vxMetersPerSecond += (ax * 0.02);
        current.vyMetersPerSecond += (ay * 0.02);
        current.omegaRadiansPerSecond = desiredSpeeds.omegaRadiansPerSecond;
        return current;
    }


    public void updateOdometryWithKinematics(){
        lastTime = currentTime;
        currentTime = Timer.getFPGATimestamp();
        totalLoopTime += (currentTime-lastTime);
        avgLoopTIme = lowpass.calculate(currentTime - lastTime);

        StatusCode signal = BaseStatusSignal.waitForAll(2 / Constants.Swerve.ODOMETRY_UPDATE_RATE_HZ_INTEGER, allOdomSignals);
        if(signal.isError()){
            ++failedOdometryUpdates;
        }
        else{
            ++successfulOdometryUpdates;
        }

        SwerveModulePosition[] positions = getModulePositions();
        SwerveModuleState[] states = getModuleStates();
        odometryLock.writeLock().lock();
        try{
            currentModulePositions = positions;
            currentRobotPose = odometry.updateWithTime(
                Timer.getFPGATimestamp(),
                getGyroHeading(),
                currentModulePositions);
            currentModuleStates = states;
        }finally{
            odometryLock.writeLock().unlock();
        }
    }


    /*
     * updateOdometryWithVision uses vision to add measurements to the odometry
     */
    public void updateOdometryWithVision(){
        int inc = 0;
        for(String cam : camNames){  
            LimelightHelpers.SetRobotOrientation(cam, getSavedPose().getRotation().getDegrees(), 0, 0, 0, 0, 0);
            LimelightHelpers.PoseEstimate mt2_estimate = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(cam);
        
            ++inc;
            // Only update pose if it is valid and if we arent spinning too fast
            if(mt2_estimate != null && mt2_estimate.tagCount != 0){//remove rotation speed limit
                SmartDashboard.putNumber(inc + " Average Tag Distance", mt2_estimate.avgTagDist);
                // Finally, we actually add the measurement to our odometry
                odometryLock.writeLock().lock();
                try{
                    odometry.addVisionMeasurement
                    (
                        mt2_estimate.pose, 
                        mt2_estimate.timestampSeconds,
                        
                        // This way it doesn't trust the rotation reading from the vision
                        // these are all the state stdevs
                        VecBuilder.fill(mt2_estimate.avgTagDist * 0.4/Units.inchesToMeters(166), mt2_estimate.avgTagDist * 0.1/Units.inchesToMeters(166), 999999999)
                    );
                }finally{
                    odometryLock.writeLock().unlock();
                }
                
                // This puts the pose reading from each camera onto the Field2d Widget,
                // Docs - https://docs.wpilib.org/en/stable/docs/software/dashboards/glass/field2d-widget.html
                m_field.getObject(cam).setPose(mt2_estimate.pose);
                SmartDashboard.putString("mt2 pose", mt2_estimate.pose.toString());
            }
        }  
    }


    /**
     * Used to find the max stator current to prevent wheel slip
     * https://pro.docs.ctr-electronics.com/en/latest/docs/hardware-reference/talonfx/improving-performance-with-current-limits.html
     * 
     * 
     * @param voltageSupplier
     */
    public void testModules(double voltage){
        //SmartDasboard.putNumber("Swerve/Module 1/Module 1 Current", modules[0].getDriveStatorCurrent());
        //SmartDashboard.putNumber("Swerve/Module 2/Module 2 Current", modules[1].getDriveStatorCurrent());
        //SmartDashboard.putNumber("Swerve/Module 3/Module 3 Current", modules[2].getDriveStatorCurrent());
        //SmartDashboard.putNumber("Swerve/Module 4/Module 4 Current", modules[3].getDriveStatorCurrent());

        for(var mod : modules){
            mod.driveWithVoltage(voltage);
        }
    }

    public Pose2d getSavedPose(){
        Pose2d pose;
        odometryLock.readLock().lock();
        try{
            pose = currentRobotPose;
        }finally{
            odometryLock.readLock().unlock();
        }
        return pose;
    }


    @Override
    public void periodic() {
        SmartDashboard.putNumber("average odometry loop time", avgLoopTIme);
        SmartDashboard.putNumber("failed odometry updates", failedOdometryUpdates);
        SmartDashboard.putNumber("sucessful odometry updates", successfulOdometryUpdates);
        SmartDashboard.putString("Robot Pose", getSavedPose().toString());

        updateOdometryWithVision();

       SmartDashboard.putNumber("NavX Position", gyro.getAngle());
       SmartDashboard.putNumber("NavX Modified Position", getGyroHeading().getDegrees());

        m_field.setRobotPose(getSavedPose());

        SwerveModuleState[] modStates = getModuleStates();

        SmartDashboard.putNumber("Module 1 Angle deg", modStates[0].angle.getDegrees());
        SmartDashboard.putNumber("Module 2 Angle deg", modStates[1].angle.getDegrees());
        SmartDashboard.putNumber("Module 3 Angle deg", modStates[2].angle.getDegrees());
        SmartDashboard.putNumber("Module 4 Angle deg", modStates[3].angle.getDegrees());
        
        
        SmartDashboard.putBoolean("Robot Rotation at Setpoint", atRotationSetpoint.getAsBoolean());
    }
}

