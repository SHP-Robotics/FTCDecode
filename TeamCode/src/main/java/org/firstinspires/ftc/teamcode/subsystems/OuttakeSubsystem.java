package org.firstinspires.ftc.teamcode.subsystems;

import static com.qualcomm.robotcore.hardware.DcMotor.RunMode.RUN_USING_ENCODER;
import static org.firstinspires.ftc.teamcode.PestoFTCConfig.INDEXER_BLOCK;
import static org.firstinspires.ftc.teamcode.PestoFTCConfig.INDEXER_OUTTAKE;
import static org.firstinspires.ftc.teamcode.subsystems.OuttakeSubsystem.OuttakeState.OUTTAKE;

import com.shprobotics.pestocore.hardware.CortexLinkedMotor;
import com.shprobotics.pestocore.hardware.CortexLinkedServo;
import com.shprobotics.pestocore.processing.MotorCortex;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.PestoFTCConfig;

public class OuttakeSubsystem {
    private final CortexLinkedMotor shooter;

    private OuttakeState state;

    private double power = PestoFTCConfig.SHOOTER_CLOSE;

    public enum OuttakeState {
        OUTTAKE,
        NEUTRAL
    }

    public OuttakeSubsystem() {
        shooter = MotorCortex.getMotor("shooter");
        shooter.setMode(RUN_USING_ENCODER);

        state = OuttakeState.NEUTRAL;
    }

    public void setState(OuttakeState state) {
        this.state = state;
    }

    public void setPower(double power) {
        assert 0 <= power && power <= 1;
        this.power = power;
    }

    public void update() {
        if (state == OUTTAKE) {
            shooter.setPowerResult(power);
        } else {
            shooter.setPowerResult(0.0);
        }
    }
}
