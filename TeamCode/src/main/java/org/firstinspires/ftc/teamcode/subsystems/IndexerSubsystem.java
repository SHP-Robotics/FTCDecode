package org.firstinspires.ftc.teamcode.subsystems;

import static org.firstinspires.ftc.teamcode.PestoFTCConfig.INDEXER_IN;
import static org.firstinspires.ftc.teamcode.PestoFTCConfig.INDEXER_OUT;
import static org.firstinspires.ftc.teamcode.subsystems.IndexerSubsystem.IndexerState.IN;
import static org.firstinspires.ftc.teamcode.subsystems.IndexerSubsystem.IndexerState.OUT;

import com.shprobotics.pestocore.hardware.CortexLinkedServo;
import com.shprobotics.pestocore.processing.MotorCortex;

public class IndexerSubsystem {
    private final CortexLinkedServo indexer;

    private IndexerState state;

    public enum IndexerState {
        IN,
        OUT
    }

    public IndexerSubsystem() {
        indexer = MotorCortex.getServo("lindex");

        state = OUT;
    }

    public void setState(IndexerState state) {
        this.state = state;
    }

    public void update() {
        if (state == OUT)
            indexer.setPositionResult(INDEXER_OUT);

        if (state == IN)
            indexer.setPositionResult(INDEXER_IN);
    }
}
