package org.firstinspires.ftc.teamcode.subsystems;

import static com.qualcomm.robotcore.hardware.DcMotor.RunMode.RUN_USING_ENCODER;
import static org.firstinspires.ftc.teamcode.PestoFTCConfig.INDEXER_BLOCK;
import static org.firstinspires.ftc.teamcode.PestoFTCConfig.INDEXER_OUTTAKE;
import static org.firstinspires.ftc.teamcode.subsystems.OuttakeSubsystem.OuttakeState.OUTTAKE;

import com.shprobotics.pestocore.hardware.CortexLinkedMotor;
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
        indexer = MotorCortex.getServo("indexer");

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
