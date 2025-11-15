package org.firstinspires.ftc.teamcode.subsystems;

import static com.qualcomm.robotcore.hardware.DcMotorSimple.Direction.FORWARD;
import static org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem.IntakeState.INTAKE;
import static org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem.IntakeState.NEUTRAL;
import static org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem.IntakeState.REJECT;

import com.shprobotics.pestocore.hardware.CortexLinkedMotor;
import com.shprobotics.pestocore.processing.MotorCortex;

public class IntakeSubsystem {
    private final CortexLinkedMotor intake;

    private IntakeState state;

    public enum IntakeState {
        INTAKE,
        NEUTRAL,
        REJECT
    }

    public IntakeSubsystem() {
        intake = MotorCortex.getMotor("intake");
        intake.setDirection(FORWARD);

        state = NEUTRAL;
    }

    public void setState(IntakeState state) {
        this.state = state;
    }

    public void update() {
        if (state == INTAKE)
            intake.setPowerResult(1.0);

        if (state == NEUTRAL)
            intake.setPowerResult(0.0);

        if (state == REJECT)
            intake.setPowerResult(-1.0);

    }
}
