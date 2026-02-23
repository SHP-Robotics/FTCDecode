package org.firstinspires.ftc.teamcode.subsystems;

import static org.firstinspires.ftc.teamcode.PestoFTCConfig.INDEXER_IN;
import static org.firstinspires.ftc.teamcode.PestoFTCConfig.INDEXER_OUT;
import static org.firstinspires.ftc.teamcode.PestoFTCConfig.INDEXER_OUTISH;
import static org.firstinspires.ftc.teamcode.subsystems.IndexerSubsystem.IndexerState.IN;
import static org.firstinspires.ftc.teamcode.subsystems.IndexerSubsystem.IndexerState.OUT;
import static org.firstinspires.ftc.teamcode.subsystems.IndexerSubsystem.IndexerState.OUTISH;

import com.shprobotics.pestocore.hardware.CortexLinkedServo;
import com.shprobotics.pestocore.processing.MotorCortex;

public class IndexerSubsystem {
    private CortexLinkedServo indexer;

    private IndexerState state;

    public enum IndexerState {
        IN,
        OUTISH,
        OUT
    }

    public IndexerSubsystem() {
        indexer = MotorCortex.getServo("lindex");
        indexer.setCachingTolerance(0.01);

        state = OUT;
    }

    public void reinitialize() {
        indexer = MotorCortex.getServo(3, 4);
        indexer.setCachingTolerance(0.01);
    }

    public void setState(IndexerState state) {
        this.state = state;
    }

    public void update() {
        if (state == OUT)
            indexer.setPositionResult(INDEXER_OUT);

        if (state == OUTISH)
            indexer.setPositionResult(INDEXER_OUTISH);

        if (state == IN)
            indexer.setPositionResult(INDEXER_IN);
    }
}
