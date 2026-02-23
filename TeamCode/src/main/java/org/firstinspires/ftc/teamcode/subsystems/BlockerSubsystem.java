package org.firstinspires.ftc.teamcode.subsystems;

import static org.firstinspires.ftc.teamcode.PestoFTCConfig.BLOCKER_BLOCK;
import static org.firstinspires.ftc.teamcode.PestoFTCConfig.BLOCKER_OUTTAKE;
import static org.firstinspires.ftc.teamcode.subsystems.BlockerSubsystem.BlockerState.BLOCK;
import static org.firstinspires.ftc.teamcode.subsystems.BlockerSubsystem.BlockerState.OUTTAKE;

import com.shprobotics.pestocore.hardware.CortexLinkedServo;
import com.shprobotics.pestocore.processing.MotorCortex;

public class BlockerSubsystem {
    private CortexLinkedServo blocker;
    private BlockerState state;

    public enum BlockerState {
        BLOCK,
        OUTTAKE
    }

    public BlockerSubsystem() {
        blocker = MotorCortex.getServo("block");
        state = BLOCK;
    }

    public void reinitialize() {
        blocker = MotorCortex.getServo(2, 4);
    }

    public void setState(BlockerState state) {
        this.state = state;
    }

    public void update() {
        if (state == BLOCK)
            blocker.setPositionResult(BLOCKER_BLOCK);
        if (state == OUTTAKE)
            blocker.setPositionResult(BLOCKER_OUTTAKE);
    }
}
