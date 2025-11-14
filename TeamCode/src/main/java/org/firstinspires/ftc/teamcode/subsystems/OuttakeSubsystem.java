package org.firstinspires.ftc.teamcode.subsystems;

import static com.qualcomm.robotcore.hardware.DcMotor.RunMode.RUN_USING_ENCODER;
import static org.firstinspires.ftc.teamcode.PestoFTCConfig.INDEXER_BLOCK;
import static org.firstinspires.ftc.teamcode.PestoFTCConfig.INDEXER_OUTTAKE;
import static org.firstinspires.ftc.teamcode.subsystems.OuttakeSubsystem.OuttakeState.OUTTAKE;

import com.shprobotics.pestocore.hardware.CortexLinkedMotor;
import com.shprobotics.pestocore.hardware.CortexLinkedServo;
import com.shprobotics.pestocore.processing.MotorCortex;

public class OuttakeSubsystem {
    private final CortexLinkedMotor shooter;
    private final CortexLinkedServo indexer;

    private OuttakeState state;

    public enum OuttakeState {
        OUTTAKE,
        NEUTRAL
    }

    public OuttakeSubsystem() {
        shooter = MotorCortex.getMotor("shooter");
        shooter.setMode(RUN_USING_ENCODER);

        indexer = MotorCortex.getServo("indexer");

        state = OuttakeState.NEUTRAL;
    }

    public void setState(OuttakeState state) {
        this.state = state;
    }

    public void update() {
        if (state == OUTTAKE) {
            shooter.setPowerResult(1.0);
            indexer.setPositionResult(INDEXER_OUTTAKE);
        } else {
            shooter.setPowerResult(0.0);
            indexer.setPositionResult(INDEXER_BLOCK);
        }
    }
}
