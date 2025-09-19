package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.networktables.IntegerSubscriber;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringSubscriber;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.DuoTalonLift.Heights;

public class SideCar extends SubsystemBase{
    
    private final NetworkTableInstance inst = NetworkTableInstance.getDefault();
    public NetworkTable sideCarTable;
    public StringSubscriber scoringLocationSub; 
    public IntegerSubscriber scoringHeight;

    public SideCar(){
        sideCarTable = inst.getTable("sidecarTable"); 
        scoringLocationSub = sideCarTable.getStringTopic("scoringLocation").subscribe("");
        scoringHeight = sideCarTable.getIntegerTopic("scoringLevel").subscribe(1);
    }

    public StringSubscriber getScoringLocation(){
        return scoringLocationSub;
    }

    public Heights getScoringLevel(){
        int integerHeight = (int)Math.round(scoringHeight.get(2));
        
        Heights height = Heights.L1; //defaults to L1
        
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
