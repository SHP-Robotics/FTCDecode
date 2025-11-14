package org.firstinspires.ftc.teamcode.subsystems;

import static org.firstinspires.ftc.teamcode.subsystems.FeederSubsystem.FeederState.FORWARD;
import static org.firstinspires.ftc.teamcode.subsystems.FeederSubsystem.FeederState.REVERSE;
import static org.firstinspires.ftc.teamcode.subsystems.FeederSubsystem.FeederState.STOPPED;

import com.shprobotics.pestocore.hardware.CortexLinkedCRServo;
import com.shprobotics.pestocore.processing.MotorCortex;

public class FeederSubsystem {
    private final CortexLinkedCRServo feeder;
    private FeederState state;

    public enum FeederState {
        FORWARD,
        REVERSE,
        STOPPED
    }

    public FeederSubsystem() {
        feeder = MotorCortex.getCRServo("feeder");
        state = STOPPED;
    }

    public void setState(FeederState state) {
        this.state = state;
    }

    public void update() {
        if (state == FORWARD)
            feeder.setPowerResult(1.0);
        if (state == REVERSE)
            feeder.setPowerResult(-1.0);
        if (state == STOPPED)
            feeder.setPowerResult(0.0);
    }
}
