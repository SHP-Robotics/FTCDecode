package org.firstinspires.ftc.teamcode.subsystems;

import static org.firstinspires.ftc.teamcode.PestoFTCConfig.INDEXER_BLOCK;
import static org.firstinspires.ftc.teamcode.PestoFTCConfig.INDEXER_OUTTAKE;

import com.shprobotics.pestocore.hardware.CortexLinkedServo;
import com.shprobotics.pestocore.processing.MotorCortex;

public class IndexerSubsystem {
    private final CortexLinkedServo indexer;

    private IndexerState state;

    public enum IndexerState {
        OUTTAKE,
        NEUTRAL
    }

    public IndexerSubsystem() {
        indexer = MotorCortex.getServo("blocker");

        state = IndexerState.NEUTRAL;
    }

    public void setState(IndexerState state) {
        this.state = state;
    }

    public void update() {
        if (state == IndexerState.OUTTAKE) {
            indexer.setPositionResult(INDEXER_OUTTAKE);
        } else {
            indexer.setPositionResult(INDEXER_BLOCK);
        }
    }
}
