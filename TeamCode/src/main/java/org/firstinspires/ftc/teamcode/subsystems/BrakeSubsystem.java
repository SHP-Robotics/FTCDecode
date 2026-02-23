package org.firstinspires.ftc.teamcode.subsystems;

import static org.firstinspires.ftc.teamcode.PestoFTCConfig.BRAKE_DOWN;
import static org.firstinspires.ftc.teamcode.PestoFTCConfig.BRAKE_UP;
import static org.firstinspires.ftc.teamcode.subsystems.BrakeSubsystem.BrakeState.DOWN;
import static org.firstinspires.ftc.teamcode.subsystems.BrakeSubsystem.BrakeState.UP;

import com.shprobotics.pestocore.hardware.CortexLinkedServo;
import com.shprobotics.pestocore.processing.MotorCortex;

public class BrakeSubsystem {
    private CortexLinkedServo brake1;
    private CortexLinkedServo brake2;

    private BrakeState state;

    public enum BrakeState {
        UP,
        DOWN
    }

    public BrakeSubsystem() {
        brake1 = MotorCortex.getServo("brake1");
        brake2 = MotorCortex.getServo("brake2");

        state = UP;
    }

    public void reinitialize() {
        brake1 = MotorCortex.getServo(0, 4);
        brake2 = MotorCortex.getServo(5, 4);
    }

    public BrakeState getState() {
        return state;
    }

    public void setState(BrakeState state) {
        this.state = state;
    }

    public void update() {
        if (state == UP) {
            brake1.setPositionResult(BRAKE_UP);
            brake2.setPositionResult(BRAKE_UP);
        }

        if (state == DOWN) {
            brake1.setPositionResult(BRAKE_DOWN);
            brake2.setPositionResult(BRAKE_DOWN);
        }
    }
}
