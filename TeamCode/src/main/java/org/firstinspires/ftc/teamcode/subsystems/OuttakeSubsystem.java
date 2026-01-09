package org.firstinspires.ftc.teamcode.subsystems;

import static com.qualcomm.robotcore.hardware.DcMotor.RunMode.RUN_USING_ENCODER;
import static org.firstinspires.ftc.teamcode.subsystems.OuttakeSubsystem.OuttakeState.OUTTAKE;

import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.shprobotics.pestocore.hardware.CortexLinkedMotor;
import com.shprobotics.pestocore.processing.MotorCortex;

import org.firstinspires.ftc.teamcode.PestoFTCConfig;

public class OuttakeSubsystem {
    private final CortexLinkedMotor lowerShooter;
    private final CortexLinkedMotor upperShooter;

    private OuttakeState state;

    private double power = PestoFTCConfig.SHOOTER_CLOSE;

    public enum OuttakeState {
        OUTTAKE,
        NEUTRAL
    }

    public OuttakeSubsystem() {
        lowerShooter = MotorCortex.getMotor("lowerShooter");
        lowerShooter.setMode(RUN_USING_ENCODER);
        lowerShooter.setDirection(DcMotorSimple.Direction.REVERSE);

        upperShooter = MotorCortex.getMotor("upperShooter");
        upperShooter.setMode(RUN_USING_ENCODER);
        upperShooter.setDirection(DcMotorSimple.Direction.FORWARD);

        state = OuttakeState.NEUTRAL;
    }

    public void setState(OuttakeState state) {
        this.state = state;
    }

    public void setPower(double power) {
        this.power = power;
    }

    public void update() {
        if (state == OUTTAKE) {
            lowerShooter.setPowerResult(power);
            upperShooter.setPowerResult(power);
        } else {
            lowerShooter.setPowerResult(0.0);
            upperShooter.setPowerResult(0.0);
        }
    }
}
