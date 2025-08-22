package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.networktables.IntegerSubscriber;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringSubscriber;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.DuoTalonLift.Heights;

public class SideCar extends SubsystemBase{
    
    public NetworkTableInstance inst;
    public NetworkTable sideCarTable;
    public StringSubscriber scoringLocationSub; 
    public IntegerSubscriber scoringHeight;


    public StringSubscriber getScoringLocation(){
        inst = NetworkTableInstance.getDefault();
        sideCarTable = inst.getTable("sidecarTable"); 
        
        return sideCarTable.getStringTopic("scoringLocation").subscribe("");
    }

    public Heights getScoringLevel(){
        inst = NetworkTableInstance.getDefault();
        sideCarTable = inst.getTable("sidecarTable"); 

        scoringHeight = sideCarTable.getIntegerTopic("scoringLevel").subscribe(1);
        int integerHeight = (int)Math.round(scoringHeight.get(2));
        
        Heights height = Heights.Ground;
        
        if (integerHeight == 1){
            height = Heights.L1;
        } else if(integerHeight == 2){
            height = Heights.L2;
        } else if(integerHeight == 3){
            height = Heights.L3;
        } else if(integerHeight == 4){
            height = Heights.L4;
        } 

        return height;
    }
}
