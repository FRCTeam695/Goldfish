
package frc.robot.subsystems;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;





public class LED extends SubsystemBase{
    AddressableLED myLED;
    AddressableLEDBuffer myLEDBuffer;
    boolean isLightOn;

    public LED(){
      
        isLightOn = true;

        myLED = new AddressableLED(1);
        myLEDBuffer = new AddressableLEDBuffer(5);
        myLED.setLength(myLEDBuffer.getLength());
        myLED.setData(myLEDBuffer);
        myLED.start();
    }

    private void LEDSet(int redAmount, int greenAmount, int blueAmount){
        AddressableLEDBuffer yourLEDBuffer = new AddressableLEDBuffer(5);
           if(isLightOn == false){
             for(var i= 0; i< myLEDBuffer.getLength(); i++){
              yourLEDBuffer.setRGB(i,redAmount, greenAmount, blueAmount);
             }
             isLightOn = true;
           }else{
              for(var i= 0; i< myLEDBuffer.getLength(); i++){
              yourLEDBuffer.setRGB(i,0, 0, 0);
             }
             isLightOn = false;
           }
       myLED.setData(yourLEDBuffer);
       myLED.start();
       }


    public Command LEDSetRed(){
        return new FunctionalCommand(
            ()-> LEDSet(255, 0, 0),

            ()-> {},

            interrupted->{},


            ()-> false,


        this);

    }
    public Command LEDSetGreen(){
        return new FunctionalCommand(
            ()-> LEDSet(0,255,0),

            ()-> {},

            interrupted->{},


            ()-> false,


        this);

    }

    public Command LEDSetBlue(){
        return new FunctionalCommand(
            ()-> LEDSet(0,0,255),

            ()-> {},

            interrupted->{},


            ()-> false,


        this);

    }


    public Command LEDSetPurple(){
        return new FunctionalCommand(
            ()-> LEDSet(255,0,255),

            ()-> {},

            interrupted->{},


            ()-> false,


        this);

    }

    public Command LEDSetOrange(){
        return new FunctionalCommand(
            ()-> LEDSet(255,25,0),

            ()-> {},

            interrupted->{},


            ()-> false,


        this);

    }

    public Command LEDSetYellow(){
        return new FunctionalCommand(
            ()-> LEDSet(255,100,0),

            ()-> {},

            interrupted->{},


            ()-> false,


        this);

    }


    private void LEDSetConfetti(){
        int size = 255;
        
        AddressableLEDBuffer yourLEDBuffer = new AddressableLEDBuffer(5);
       
            

           if(isLightOn == false){

            
            
             for(var i= 0; i< myLEDBuffer.getLength(); i++){
                int randomRed = (int)(Math.random()*(size+1));
                int randomGreen = (int)(Math.random()*(size+1));
                int randomBlue = (int)(Math.random()*(size+1));
              yourLEDBuffer.setRGB(i,randomRed,randomGreen,randomBlue );
             }
             isLightOn = true;
             
            
        
           }else{
              for(var i= 0; i< myLEDBuffer.getLength(); i++){
              yourLEDBuffer.setRGB(i,0, 0, 0);
             }
             isLightOn = false;
           }
       myLED.setData(yourLEDBuffer);
       myLED.start();



       }

        public Command Confetti(){
            
            return new FunctionalCommand(
                ()-> {},
    
                ()-> {
                    LEDSetConfetti();

                },
    
                interrupted->{},
    
    
                ()-> false,
    
    
            this);
    }



    





   


}