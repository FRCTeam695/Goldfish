package frc.robot.subsystems;

import java.util.HashMap;
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
        HashMap<String, Heights> heightsMap = new HashMap<>();
        
        heightsMap.put("1", Heights.L1);
        heightsMap.put("2", Heights.L2);
        heightsMap.put("3", Heights.L3);
        heightsMap.put("4", Heights.L4);

        Heights height = heightsMap.get(Integer.toString(integerHeight)); //defaults to ground level

        return height;
    }
}
