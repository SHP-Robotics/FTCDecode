package org.firstinspires.ftc.teamcode.subsystems;

import static com.qualcomm.robotcore.hardware.DcMotor.RunMode.RUN_USING_ENCODER;
import static org.firstinspires.ftc.teamcode.PestoFTCConfig.SHOOTER_KP;
import static org.firstinspires.ftc.teamcode.subsystems.OuttakeSubsystem.OuttakeState.OUTTAKE;

import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.shprobotics.pestocore.algorithms.PID;
import com.shprobotics.pestocore.hardware.CortexLinkedMotor;
import com.shprobotics.pestocore.processing.MotorCortex;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.PestoFTCConfig;

public class OuttakeSubsystem {
    private final CortexLinkedMotor lowerShooter;
    private final CortexLinkedMotor upperShooter;
    private final PID pidController;

    private OuttakeState state;

    private double ffPower = PestoFTCConfig.SHOOTER_FF_CLOSE;
    private double expectedRPM = PestoFTCConfig.SHOOTER_CLOSE;

    public enum OuttakeState {
        OUTTAKE,
        NEUTRAL
    }

    public OuttakeSubsystem() {
        lowerShooter = MotorCortex.getMotor("lowerShooter");
        lowerShooter.setMode(RUN_USING_ENCODER);
        lowerShooter.setDirection(DcMotorSimple.Direction.REVERSE);

        upperShooter = MotorCortex.getMotor("upperShooter");
        upperShooter.setMode(RUN_USING_ENCODER);
        upperShooter.setDirection(DcMotorSimple.Direction.FORWARD);

        pidController = new PID(SHOOTER_KP, 0, 0);

        state = OuttakeState.NEUTRAL;
    }

    public void setState(OuttakeState state) {
        this.state = state;
    }

    public void setFFPower(double ffPower) {
        this.ffPower = ffPower;
    }

    public void setRPM(double expectedRPM) {
        this.expectedRPM = expectedRPM;
    }

    public double getRPM() {
        return lowerShooter.getVelocity(AngleUnit.RADIANS);
    }

    public double getTargetRPM() {
        return expectedRPM;
    }

    public boolean isBusy() {
        return Math.abs(this.getRPM() - expectedRPM) >= PestoFTCConfig.SHOOTER_RPM_TOLERANCE;
    }

    public void update() {
        if (state == OUTTAKE) {
            double rpm = getRPM();
            double power = ffPower + pidController.getOutput(rpm, expectedRPM);
            power = Math.max(power, 0);

            lowerShooter.setPowerResult(power);
            upperShooter.setPowerResult(power);
        } else {
            lowerShooter.setPowerResult(0.0);
            upperShooter.setPowerResult(0.0);
        }
    }
}
