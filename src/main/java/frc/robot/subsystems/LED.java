package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.util.Color;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Seconds;

import java.util.function.DoubleSupplier;

public class LED extends SubsystemBase {
    private double xaxis; 

    private int[] LEDShift = {0, 1, 1, 1, 0};
    private long counter = 0;

    //this stuff above isn't important

    private AddressableLED realLED;
    private AddressableLEDBuffer realLEDBuffer;

    public LED() {

        realLED = new AddressableLED(1);
        realLEDBuffer = new AddressableLEDBuffer(5);

        realLED.setLength(realLEDBuffer.getLength());
        realLED.setData(realLEDBuffer);
        realLED.start();

    }
    //Solid Yellow
    public void setYellow() {
        LEDPattern red = LEDPattern.solid(Color.kYellow);

        red.applyTo(realLEDBuffer);

        realLED.setData(realLEDBuffer);
    }
    //Flashing Yellow
    public void breathingYellow() {
        LEDPattern base = LEDPattern.solid(Color.kYellow);
        LEDPattern pattern = base.breathe(Seconds.of(2));

        pattern.applyTo(realLEDBuffer);
        realLED.setData(realLEDBuffer);
    }
    //Rainbow Pattern
    public void rainbow() {
        LEDPattern rainbow = LEDPattern.rainbow(255, 255); //The first value is saturation, second value is brightness. Max of 255
        /*
         * Calculate this by finding the length of your LED Strip. 
         * Divide 100 by that number and then multiply by the number of leds on your strip that you measured. 
         * Then divide that number by one. 
         * That number represents the amount of LEDs per meter of strip, 
         * which in this case is rounded to 58.8. 
         * Dividing one by this number is to create a ratio that says "for every one meter, there are 58.8 LEDs"
         */
        Distance kLedSpacing = Meters.of(1 / 58.8); 
        LEDPattern scrollingRainbow = rainbow.scrollAtAbsoluteSpeed(MetersPerSecond.of(0.1), kLedSpacing); //creates scrolling rainbow effect. Change the MetersPerSecond.of() variable to control speed.

        scrollingRainbow.applyTo(realLEDBuffer);
        realLED.setData(realLEDBuffer);
    }
    //Turns LED Off
    public void setLEDOff() {
        for (int i = 0; i < realLEDBuffer.getLength(); i++) {

            realLEDBuffer.setRGB(i, 0, 0, 0);

        }

        realLED.setData(realLEDBuffer);
    }

    // ------------------------------------------------------------------------------------
    //random method that isn't used
    public void setColor(int colorNumber) {

        if (colorNumber == 0) {
            setYellow();
        }
        if (colorNumber == 1) {
            rainbow();
        }
    }
    //breatheYellow command
    public Command breatheYellow() {
        return new FunctionalCommand(

                () -> {

                },

                () -> {
                    breathingYellow();
                },

                interrupted -> {
                    setLEDOff();
                },

                () -> false,

                this);
    }
    //rainbowLED command
    public Command rainbowLED() {
        return new FunctionalCommand(
            () -> {}, 
            
            () -> {
                rainbow();
            }, 
        
            interrupted -> {
                setLEDOff();
            }, 
            
            () -> false, 
        
        this);
    }
}