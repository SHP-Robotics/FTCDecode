package org.firstinspires.ftc.teamcode.subsystems;

import static org.firstinspires.ftc.teamcode.PestoFTCConfig.BLOCKER_BLOCK;
import static org.firstinspires.ftc.teamcode.PestoFTCConfig.BLOCKER_OUTTAKE;

import com.shprobotics.pestocore.hardware.CortexLinkedServo;
import com.shprobotics.pestocore.processing.MotorCortex;

public class BlockerSubsystem {
    private CortexLinkedServo blocker;

    private BlockerState state;

    public enum BlockerState{
        BLOCK,
        NEUTRAL
    }

    public BlockerSubsystem(){
        blocker = MotorCortex.getServo("blocker");
        state = BlockerState.BLOCK;
    }

    public void setState(BlockerState state){this.state = state;}

    public void update(){
        if (state == BlockerState.BLOCK){
            blocker.setPositionResult(BLOCKER_BLOCK);
        }
        else{
            blocker.setPositionResult(BLOCKER_OUTTAKE);
        }
    }
}
