package org.firstinspires.ftc.teamcode.subsystems;

import static org.firstinspires.ftc.teamcode.subsystems.OuttakeSubsystem.OuttakeState.OUTTAKE;

import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.shprobotics.pestocore.hardware.CortexLinkedMotor;
import com.shprobotics.pestocore.processing.MotorCortex;

public class OuttakeSubsystem {
    private final CortexLinkedMotor shooterLeft;
    private final CortexLinkedMotor shooterRight;

    private OuttakeState state;

    public enum OuttakeState {
        OUTTAKE,
        NEUTRAL
    }

    private double power;

    public OuttakeSubsystem() {
        shooterLeft = MotorCortex.getMotor("spinnerLeft");
        shooterRight = MotorCortex.getMotor("spinnerRight");

        shooterLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        shooterRight.setDirection(DcMotorSimple.Direction.REVERSE);

//        shooterLeft.setMode(RUN_USING_ENCODER);
//        shooterRight.setMode(RUN_USING_ENCODER);

        state = OuttakeState.NEUTRAL;

        power = 0.4;
    }

    public void setPower(double power) {
        this.power = power;
    }

    public void setState(OuttakeState state) {
        this.state = state;
    }

    public void update() {
        if (state == OUTTAKE) {
            shooterLeft.setPowerResult(power);
            shooterRight.setPowerResult(power);
        } else {
            shooterLeft.setPowerResult(0.0);
            shooterRight.setPowerResult(0.0);
        }
    }
}
