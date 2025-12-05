package org.firstinspires.ftc.teamcode.subsystems;

import static org.firstinspires.ftc.teamcode.PestoFTCConfig.HOOD_AUTO_FAR;
import static org.firstinspires.ftc.teamcode.PestoFTCConfig.HOOD_CLOSE;
import static org.firstinspires.ftc.teamcode.PestoFTCConfig.HOOD_FAR;
import static org.firstinspires.ftc.teamcode.PestoFTCConfig.HOOD_MID;
import static org.firstinspires.ftc.teamcode.subsystems.HoodSubsystem.HoodState.AUTO_FAR;
import static org.firstinspires.ftc.teamcode.subsystems.HoodSubsystem.HoodState.CLOSE;
import static org.firstinspires.ftc.teamcode.subsystems.HoodSubsystem.HoodState.FAR;
import static org.firstinspires.ftc.teamcode.subsystems.HoodSubsystem.HoodState.MID;

import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.shprobotics.pestocore.hardware.CortexLinkedServo;
import com.shprobotics.pestocore.processing.MotorCortex;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class HoodSubsystem {
    private final CortexLinkedServo hood;
    private final CortexLinkedServo led;
    private final DistanceSensor distanceSensor;

    private HoodState state;

    public enum HoodState {
        CLOSE,
        MID,
        FAR,
        AUTO_FAR
    }

    public HoodSubsystem() {
        hood = MotorCortex.getServo("hood");
        led = MotorCortex.getServo("led");

        distanceSensor = (DistanceSensor) MotorCortex.hardwareMap.get("distance");

        state = CLOSE;
    }

    public HoodState getState() {
        return state;
    }

    public void setState(HoodState state) {
        this.state = state;
    }

    public double getDistance() {
        return distanceSensor.getDistance(DistanceUnit.CM);
    }

    public void update() {
        boolean ledOverride = distanceSensor.getDistance(DistanceUnit.CM) < 10;

        if (ledOverride)
            led.setPositionResult(0.39); // YALLOW

        if (state == CLOSE) {
            hood.setPositionResult(HOOD_CLOSE);
            if (!ledOverride)
                led.setPositionResult(0.5); // GREEN
        }

        if (state == MID) {
            hood.setPositionResult(HOOD_MID);
            if (!ledOverride)
                led.setPositionResult(0.61); // BLUE
        }

        if (state == FAR) {
            hood.setPositionResult(HOOD_FAR);
            if (!ledOverride)
                led.setPositionResult(0.28); // RED
        }

        if (state == AUTO_FAR) {
            hood.setPositionResult(HOOD_AUTO_FAR);
            if (!ledOverride)
                led.setPositionResult(0.28); // RED
        }
    }
}
