package org.firstinspires.ftc.teamcode.subsystems;

import static com.qualcomm.robotcore.hardware.DcMotor.RunMode.RUN_USING_ENCODER;
import static com.qualcomm.robotcore.hardware.DcMotor.RunMode.STOP_AND_RESET_ENCODER;

import static org.firstinspires.ftc.teamcode.subsystems.TurretSubsystem.TurretState.CUSTOM_POSITION;
import static org.firstinspires.ftc.teamcode.subsystems.TurretSubsystem.TurretState.LEFT;
import static org.firstinspires.ftc.teamcode.subsystems.TurretSubsystem.TurretState.MANUAL;
import static org.firstinspires.ftc.teamcode.subsystems.TurretSubsystem.TurretState.RIGHT;
import static org.firstinspires.ftc.teamcode.subsystems.TurretSubsystem.TurretState.STRAIGHT;

import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.shprobotics.pestocore.algorithms.PID;
import com.shprobotics.pestocore.hardware.CortexLinkedMotor;
import com.shprobotics.pestocore.processing.MotorCortex;

import org.firstinspires.ftc.teamcode.PestoFTCConfig;

public class TurretSubsystem {
    private final CortexLinkedMotor turret;
    private PID pidController;

    private TurretState state;

    private double customPosition;

    public enum TurretState {
        LEFT,
        STRAIGHT,
        RIGHT,

        CUSTOM_POSITION,
        MANUAL
    }

    public TurretSubsystem() {
        turret = MotorCortex.getMotor("turret");
        turret.setDirection(DcMotorSimple.Direction.REVERSE);
        turret.setMode(RUN_USING_ENCODER);

        pidController = new PID(PestoFTCConfig.TURRET_KP, 0, 0);

        state = STRAIGHT;
    }

    public void setState(TurretState state) {
        this.state = state;
    }

    public void setPower(double power) {
        assert state == MANUAL;
        turret.setPowerResult(power);
    }

    public void setPosition(double position) {
        this.customPosition = position;
        this.state = CUSTOM_POSITION;
    }

    public double getPosition() {
        return turret.getCurrentPosition();
    }

    public TurretState getState() {
        return state;
    }

    public void rezero() {
        turret.setMode(STOP_AND_RESET_ENCODER);
        turret.setMode(RUN_USING_ENCODER);
    }

    public void update() {
        if (this.state == MANUAL)
            return;

        double targetPosition = 0;

        if (this.state == LEFT)
            targetPosition = PestoFTCConfig.TURRET_LEFT;

        if (this.state == RIGHT)
            targetPosition = PestoFTCConfig.TURRET_RIGHT;

        if (this.state == CUSTOM_POSITION)
            targetPosition = customPosition;

        if (Math.abs(turret.getCurrentPosition() - targetPosition) < 6) {
            turret.setPowerResult(0.0);
            return;
        }

        double power = pidController.getOutput(turret.getCurrentPosition(), targetPosition);

        power += Math.signum(power) * PestoFTCConfig.TURRET_STATIC;

        turret.setPowerResult(power);
    }
}
