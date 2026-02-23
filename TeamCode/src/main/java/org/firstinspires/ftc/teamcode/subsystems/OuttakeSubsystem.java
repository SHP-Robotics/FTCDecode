package org.firstinspires.ftc.teamcode.subsystems;

import static com.qualcomm.robotcore.hardware.DcMotor.RunMode.RUN_USING_ENCODER;
import static org.firstinspires.ftc.teamcode.PestoFTCConfig.SHOOTER_KD;
import static org.firstinspires.ftc.teamcode.PestoFTCConfig.SHOOTER_KP;
import static org.firstinspires.ftc.teamcode.subsystems.OuttakeSubsystem.OuttakeState.OUTTAKE;

import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.shprobotics.pestocore.algorithms.PID;
import com.shprobotics.pestocore.hardware.CortexLinkedMotor;
import com.shprobotics.pestocore.processing.MotorCortex;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.PestoFTCConfig;

public class OuttakeSubsystem {
    private CortexLinkedMotor leftShooter;
    private CortexLinkedMotor rightShooter;
    private final PID pidController;

    private OuttakeState state;

    private double ffPower = PestoFTCConfig.SHOOTER_FF_CLOSE;
    private double expectedRPM = PestoFTCConfig.SHOOTER_CLOSE;

    public enum OuttakeState {
        OUTTAKE,
        NEUTRAL
    }

    public OuttakeSubsystem() {
        leftShooter = MotorCortex.getMotor("leftShooter");
        leftShooter.setMode(RUN_USING_ENCODER);
        leftShooter.setDirection(DcMotorSimple.Direction.REVERSE);

        rightShooter = MotorCortex.getMotor("rightShooter");
        rightShooter.setMode(RUN_USING_ENCODER);
        rightShooter.setDirection(DcMotorSimple.Direction.FORWARD);

        pidController = new PID(SHOOTER_KP, 0, SHOOTER_KD);

        state = OuttakeState.NEUTRAL;
    }

    public void reinitialize() {
        leftShooter = MotorCortex.getMotor(0, 2);
        leftShooter.setMode(RUN_USING_ENCODER);
        leftShooter.setDirection(DcMotorSimple.Direction.FORWARD);

        rightShooter = MotorCortex.getMotor(1, 2);
        rightShooter.setMode(RUN_USING_ENCODER);
        rightShooter.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    public void setState(OuttakeState state) {
        this.state = state;
    }

    public void setPowerDirect(double power) {
        leftShooter.setPowerResult(power);
        rightShooter.setPowerResult(power);
    }

    public void setFFPower(double ffPower) {
        this.ffPower = ffPower;
    }

    public void setRPM(double expectedRPM) {
        this.expectedRPM = expectedRPM;
    }

    public double getRPM() {
        return leftShooter.getVelocity(AngleUnit.RADIANS);
    }

    public double getTargetRPM() {
        return expectedRPM;
    }

    public boolean isBusy() {
        return Math.abs(this.getRPM() - expectedRPM) >= PestoFTCConfig.SHOOTER_RPM_TOLERANCE;
    }

    public double getRadians(double metersPerSecond) {
        return metersPerSecond * 10;
    }

    public double prepShooter(double distance, double targetHeight) {
        double GRAVITY = -9.81;

        // TODO: tune
        double shootingAngle = Math.toRadians(50);
        double cosAngle = Math.cos(shootingAngle);
        double sinAngle = Math.sin(shootingAngle);
        double tanAngle = Math.tan(shootingAngle);

        double v = Math.sqrt((-GRAVITY * distance * distance) / (2 * cosAngle * cosAngle * (distance * tanAngle - targetHeight)));

        return v;
    }

    public void update() {
        if (state == OUTTAKE) {
            double rpm = getRPM();
            double power = ffPower + pidController.getOutput(rpm, expectedRPM);
            power = Math.max(power, 0);

            leftShooter.setPowerResult(power);
            rightShooter.setPowerResult(power);
        } else {
            leftShooter.setPowerResult(0.0);
            rightShooter.setPowerResult(0.0);
        }
    }
}
