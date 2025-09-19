package frc.BisonLib.BaseProject;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.hardware.TalonFXS;

import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.PubSubOption;

public class Utilities {
    public static DoublePublisher getPubForTopic50Hz(NetworkTable tab, String name) {
        return tab.getDoubleTopic(name).publish(PubSubOption.periodic(0.02));
    }

    public static Runnable runTalonFXSDutyCycle(TalonFXS motor, DoubleSupplier dutyCycle)
    {
        DutyCycleOut setpoint = new DutyCycleOut(0);
        return () -> {
            setpoint.Output = dutyCycle.getAsDouble();
            motor.setControl(setpoint);
        };
    }

    public static void publish(DoublePublisher pub, BaseStatusSignal sup) {
        pub.set(sup.getValueAsDouble());
    }

}
