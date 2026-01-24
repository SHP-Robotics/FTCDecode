package org.firstinspires.ftc.teamcode.subsystems;

import static org.firstinspires.ftc.teamcode.PestoFTCConfig.HOOD_CLOSE;
import static org.firstinspires.ftc.teamcode.PestoFTCConfig.HOOD_FAR;
import static org.firstinspires.ftc.teamcode.PestoFTCConfig.HOOD_MID;

import com.shprobotics.pestocore.hardware.CortexLinkedServo;
import com.shprobotics.pestocore.processing.MotorCortex;

public class HoodSubsystem {
    private CortexLinkedServo hood;
    private HoodState state;
    public enum HoodState{
        CLOSE,
        MID,
        FAR
    }

    public HoodSubsystem(){
        hood = MotorCortex.getServo("hood");
        state = HoodState.CLOSE;
    }

    public void setState(HoodState state){this.state = state;}

    public HoodState getState(){return this.state;}
    public void update(){
        if(state == HoodState.CLOSE){
            hood.setPositionResult(HOOD_CLOSE);
        }
        else if(state == HoodState.MID){
            hood.setPositionResult(HOOD_MID);
        }
        else{
            hood.setPositionResult(HOOD_FAR);
        }
    }
}
