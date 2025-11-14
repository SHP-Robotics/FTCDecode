package org.firstinspires.ftc.teamcode.subsystems;

import static org.firstinspires.ftc.teamcode.PestoFTCConfig.HOOD_CLOSE;
import static org.firstinspires.ftc.teamcode.PestoFTCConfig.HOOD_FAR;
import static org.firstinspires.ftc.teamcode.PestoFTCConfig.HOOD_MID;
import static org.firstinspires.ftc.teamcode.subsystems.HoodSubsystem.HoodState.CLOSE;
import static org.firstinspires.ftc.teamcode.subsystems.HoodSubsystem.HoodState.FAR;
import static org.firstinspires.ftc.teamcode.subsystems.HoodSubsystem.HoodState.MID;

import com.shprobotics.pestocore.hardware.CortexLinkedServo;
import com.shprobotics.pestocore.processing.MotorCortex;

public class HoodSubsystem {
    private final CortexLinkedServo hood;
    private final CortexLinkedServo led;

    private HoodState state;

    public enum HoodState {
        CLOSE,
        MID,
        FAR
    }

    public HoodSubsystem() {
        hood = MotorCortex.getServo("hood");
        led = MotorCortex.getServo("led");

        state = CLOSE;
    }

    public HoodState getState() {
        return state;
    }

    public void setState(HoodState state) {
        this.state = state;
    }

    public void update() {
        if (state == CLOSE) {
            hood.setPositionResult(HOOD_CLOSE);
            led.setPositionResult(0.5); // GREEN
        }

        if (state == MID) {
            hood.setPositionResult(HOOD_MID);
            led.setPositionResult(0.61); // BLUE
        }

        if (state == FAR) {
            hood.setPositionResult(HOOD_FAR);
            led.setPositionResult(0.28); // RED
        }
    }
}
