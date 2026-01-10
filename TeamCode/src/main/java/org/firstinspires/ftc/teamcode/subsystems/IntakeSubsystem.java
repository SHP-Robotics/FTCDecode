package org.firstinspires.ftc.teamcode.subsystems;

import static com.qualcomm.robotcore.hardware.DcMotorSimple.Direction.REVERSE;
import static org.firstinspires.ftc.teamcode.PestoFTCConfig.DROPDOWN_DRIVE;
import static org.firstinspires.ftc.teamcode.PestoFTCConfig.DROPDOWN_INTAKE;
import static org.firstinspires.ftc.teamcode.PestoFTCConfig.DROPDOWN_PUSH;
import static org.firstinspires.ftc.teamcode.PestoFTCConfig.DROPDOWN_PUSH_AUTO;
import static org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem.IntakeState.INTAKE;
import static org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem.IntakeState.NEUTRAL;
import static org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem.IntakeState.OUTTAKE;
import static org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem.IntakeState.OUTTAKE_AUTO;
import static org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem.IntakeState.REJECT;

import com.qualcomm.hardware.rev.Rev9AxisImu;
import com.shprobotics.pestocore.hardware.CortexLinkedCRServo;
import com.shprobotics.pestocore.hardware.CortexLinkedMotor;
import com.shprobotics.pestocore.processing.MotorCortex;

public class IntakeSubsystem {
    private final CortexLinkedMotor intake;
    public final CortexLinkedCRServo dropdown;

    public final Rev9AxisImu imu;
    private double pitchVelocity = 0.0;
    private double pitch = 0.0;
    private double time = 0.0;
    private double power = 0.0;
    public double dropdownTarget;

    private IntakeState state;

    public enum IntakeState {
        INTAKE,
        NEUTRAL,
        REJECT,
        OUTTAKE,
        OUTTAKE_AUTO
    }

    public IntakeSubsystem() {
        intake = MotorCortex.getMotor("intake");
        intake.setDirection(REVERSE);

        imu = (Rev9AxisImu) MotorCortex.hardwareMap.get("ext_imu");

        dropdown = MotorCortex.getCRServo("dropdown");

        state = NEUTRAL;
        dropdownTarget = DROPDOWN_DRIVE;
    }

    public void setState(IntakeState state) {
        this.state = state;
    }

    public void update() {
        if (state == INTAKE) {
            intake.setPowerResult(1.0);
            dropdownTarget = DROPDOWN_INTAKE;
        }

        if (state == NEUTRAL) {
            intake.setPowerResult(0.15);
            dropdownTarget = DROPDOWN_DRIVE;
        }

        if (state == REJECT) {
            intake.setPowerResult(-1.0);
            dropdownTarget = DROPDOWN_INTAKE;
        }

        if (state == OUTTAKE) {
            intake.setPowerResult(1.0);
            dropdownTarget = DROPDOWN_PUSH;
        }

        if (state == OUTTAKE_AUTO) {
            intake.setPowerResult(1.0);
            dropdownTarget = DROPDOWN_PUSH_AUTO;
        }

        pitchVelocity = (imu.getRobotYawPitchRollAngles().getPitch() - pitch) / ((System.nanoTime() - time) / 1E9);

        // dropdown PID logic
        pitch = imu.getRobotYawPitchRollAngles().getPitch();
        time = System.nanoTime();

        // power save with <10 degrees of error
        if (Math.abs(dropdownTarget - pitch) < 10) {
            dropdown.setPowerResult(0.0);
            return;
        }

        double error = 0.01 * (dropdownTarget - pitch) - 0.003 * pitchVelocity;

        power = Math.min(1.0, Math.max(-1.0, error));

        dropdown.setPowerResult(-power);
    }
}
