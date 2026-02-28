package org.firstinspires.ftc.teamcode.subsystems;

import static com.qualcomm.robotcore.hardware.DcMotor.RunMode.RUN_USING_ENCODER;

import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.shprobotics.pestocore.hardware.CortexLinkedMotor;
import com.shprobotics.pestocore.processing.MotorCortex;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

public class OuttakeSubsystem {
    private CortexLinkedMotor leftShooter;
    private CortexLinkedMotor rightShooter;

    private OuttakeState state;

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

    public double getRPM() {
        return leftShooter.getVelocity(AngleUnit.RADIANS);
    }

    public double getVelocity() {
        return Math.abs(leftShooter.getVelocity());
    }

    public double getExpectedVelocity(double power) {
        return leftShooter.getMotorType().getAchieveableMaxTicksPerSecondRounded() * Math.abs(power);
    }

    public boolean isBusy(double power) {
        double expectedTPS = getExpectedVelocity(power);
        double TPS = getVelocity();

        return (expectedTPS - TPS) / expectedTPS > 0.03;
    }
}
