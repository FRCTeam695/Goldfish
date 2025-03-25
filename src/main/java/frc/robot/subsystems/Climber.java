package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Climber extends SubsystemBase{
    private TalonFX m_talon;
    private TalonFXConfiguration m_configs;

    public Climber() {
        m_talon = new TalonFX(56); // ID?
        m_configs = new TalonFXConfiguration();

        //m_configs.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
        m_configs.SoftwareLimitSwitch.ForwardSoftLimitThreshold = 0; // Arbitrary numbers for rn
        //m_configs.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
        m_configs.SoftwareLimitSwitch.ReverseSoftLimitThreshold = 0;

        m_talon.getConfigurator().apply(m_configs);
        m_talon.setPosition(0);
    }

    private Command runClimb(double value) {
        DutyCycleOut output = new DutyCycleOut(0);
        return run(() -> {
            output.Output = value; // Speed of climb (Random value for rn)
            m_talon.setControl(output);
        });
    }
    public Command climbOut() {
        return runClimb(0.5);
    }
    public Command climbIn() {
        return runClimb(-0.5);
    }
}