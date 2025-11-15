package org.firstinspires.ftc.teamcode.subsystems;

import com.shprobotics.pestocore.hardware.CortexLinkedCRServo;
import com.shprobotics.pestocore.processing.MotorCortex;

public class FeederSubsystem {
    private final CortexLinkedCRServo feederLeft;
    private final CortexLinkedCRServo feederRight;

    private FeederState state;

    public enum FeederState {
        INTAKE,
        NEUTRAL,
        REJECT
    }

    public FeederSubsystem() {
        feederLeft = MotorCortex.getCRServo("feederLeft");
        feederRight = MotorCortex.getCRServo("feederRight");

        state = FeederState.NEUTRAL;
    }

    public void setState(FeederState state) {
        this.state = state;
    }

    public void update() {
        if (state == FeederState.INTAKE) {
            feederLeft.setPowerResult(0.5);
            feederRight.setPowerResult(-0.5);
        }

        if (state == FeederState.NEUTRAL) {
            feederLeft.setPowerResult(0.0);
            feederRight.setPowerResult(0.0);
        }

        if (state == FeederState.REJECT) {
            feederLeft.setPowerResult(-1.0);
            feederRight.setPowerResult(1.0);
        }
    }
}
