package org.firstinspires.ftc.teamcode.subsystems;

import static org.firstinspires.ftc.teamcode.subsystems.HoodSubsystem.HoodState.CLOSE;

import com.shprobotics.pestocore.hardware.CortexLinkedServo;
import com.shprobotics.pestocore.processing.MotorCortex;

public class HoodSubsystem {
    private CortexLinkedServo hood;

    private HoodState state;

    public enum HoodState {
        CLOSE,
        MID,
        FAR
    }

    public HoodSubsystem() {
        hood = MotorCortex.getServo("hood");

        state = CLOSE;
    }

    public void reinitialize() {
        hood = MotorCortex.getServo(1, 4);
    }

    public HoodState getState() {
        return state;
    }

    public void setState(HoodState state) {
        this.state = state;
    }

    public void setAngleDirect(double angle) {
        hood.setPositionResult(angle);
    }

    public void update() {
    }
}
