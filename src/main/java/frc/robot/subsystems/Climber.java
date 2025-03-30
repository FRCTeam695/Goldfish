package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class Climber extends SubsystemBase{
    private TalonFX climberMotor;
    private TalonFXConfiguration m_configs;
    
    public Trigger closeToReverseLimit;
    public Trigger closeToForwardLimit;

    public Climber() {
        climberMotor = new TalonFX(56); // ID?
        m_configs = new TalonFXConfiguration();

        m_configs.SoftwareLimitSwitch.ForwardSoftLimitEnable = false;
        m_configs.SoftwareLimitSwitch.ForwardSoftLimitThreshold = 170;
        m_configs.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
        m_configs.SoftwareLimitSwitch.ReverseSoftLimitThreshold = 0;
        m_configs.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        climberMotor.getConfigurator().apply(m_configs);
        climberMotor.setPosition(0);

        closeToReverseLimit = new Trigger(()-> climberMotor.getPosition().getValueAsDouble() < 1);
        closeToForwardLimit = new Trigger(()-> climberMotor.getPosition().getValueAsDouble() > 169);
    }

    private Command climbDutyCycle(double value) {
        DutyCycleOut output = new DutyCycleOut(0);
        return run(() -> {
            output.Output = value;
            climberMotor.setControl(output);
        });
    }

    public Command runClimb(double speed) {
        return climbDutyCycle(speed).finallyDo(()->climberMotor.setControl(new DutyCycleOut(0)));
    }
    public Command climbInNoSoftlimits() {
        return 
        runOnce(()->{
            m_configs = new TalonFXConfiguration();

            m_configs.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
            m_configs.SoftwareLimitSwitch.ForwardSoftLimitThreshold = 170;
            m_configs.SoftwareLimitSwitch.ReverseSoftLimitEnable = false;
            m_configs.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    
            climberMotor.getConfigurator().apply(m_configs);
        }).andThen(
            runClimb(-0.3)
        ).finallyDo(()->climberMotor.setControl(new DutyCycleOut(0)));
    }

    public Command climbOutNoSoftLimits() {
        return 
        runOnce(()->{
            m_configs = new TalonFXConfiguration();

            m_configs.SoftwareLimitSwitch.ForwardSoftLimitEnable = false;
            m_configs.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
            m_configs.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    
            climberMotor.getConfigurator().apply(m_configs);
        }).andThen(
            runClimb(0.1)
        ).finallyDo(()->climberMotor.setControl(new DutyCycleOut(0)));
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Climber Position", climberMotor.getPosition().getValueAsDouble());
        SmartDashboard.putBoolean("Climber Close to Zero", closeToReverseLimit.getAsBoolean());
    }
}