package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

@TeleOp(name = "TestOutakePowers")
public class TestOutakePowers extends BaseRobot {
    DcMotor lI;
    DcMotor rI;

    DcMotor Out;
    DcMotor Boost;
    Servo Push;

    AprilTagProcessor AT;
    VisionPortal visionPortal;
    WebcamName Eyes;

    double turnRatio = 0.75; //time over rotation angle ratio
    double strafeRatio = 1; //time over distance ratio
    double moveRatio = 0.5; //time over distance ratio
    double disFromAT = 40; //inches in depth from April Tag
    double rotationSpeedMultiplier = 1; //multiplier on rotation speed
    boolean rBumperStateCheck = false;
    boolean lBumperStateCheck = false;

    public void turn_180 (double turnRatio) {
        // Turn 180 so output is facing target
        teleOpController.driveRobotCentric(0, 0, 0.5);
        sleep((int) Math.round(180*turnRatio));
        teleOpController.driveRobotCentric(0,0,0);
    }
    public void fire_w_power (double powerOut, double powerBoost) {
        Out.setPower(powerOut);
        Boost.setPower(powerBoost);
        lI.setPower(1);
        rI.setPower(-1);
        Push.setPosition(0.7);

        sleep(100);

        Out.setPower(0);
        Boost.setPower(0);
        lI.setPower(0);
        rI.setPower(0);
        Push.setPosition(0);
    }
    public void use_targeting_system() {
        List<AprilTagDetection> detections = AT.getDetections();
        for (AprilTagDetection detection : detections) {
            if (detection.metadata != null && (detection.id == 20 || detection.id == 24)) {

                // Get coordinate value distances from April Tag
                double angleOff = detection.ftcPose.yaw;
                double xD = detection.ftcPose.x;
                double yD = detection.ftcPose.y;

                // Rotate to square up with April Tag
                teleOpController.driveRobotCentric(0, 0, 0.5 * Math.signum(angleOff));
                sleep((int) Math.round((Math.abs(angleOff)) * turnRatio));
                teleOpController.driveRobotCentric(0, 0, 0);

                // Move so directly __ inches from April Tag
                // (align x)
                teleOpController.driveRobotCentric(0, 0.5 * Math.signum(xD), 0);
                sleep((int) Math.round((Math.abs(xD)) * strafeRatio));
                teleOpController.driveRobotCentric(0, 0, 0);

                // (align y)
                teleOpController.driveRobotCentric(0.5 * Math.signum(yD - disFromAT), 0, 0);
                sleep((int) Math.round((Math.abs(yD - disFromAT)) * moveRatio));
                teleOpController.driveRobotCentric(0, 0, 0);
            }
        }
    }

    @Override
    public void runOpMode() {
        super.runOpMode();


        lI = hardwareMap.get(DcMotor.class, "left");
        lI.setDirection(DcMotorSimple.Direction.REVERSE);
        rI = hardwareMap.get(DcMotor.class, "right");
        rI.setDirection(DcMotorSimple.Direction.REVERSE);

        Out = hardwareMap.get(DcMotor.class, "out");
        Boost = hardwareMap.get(DcMotor.class, "boost");
        Boost.setDirection(DcMotorSimple.Direction.REVERSE);
        Push = hardwareMap.get(Servo.class, "push");

        double OutPower = 0.4;
        double BoostPower = 0.4;


        waitForStart();

        teleOpController.resetIMU(180);

        while (opModeIsActive() && !isStopRequested()) {

            teleOpController.driveFieldCentric(-gamepad1.left_stick_y, gamepad1.left_stick_x, rotationSpeedMultiplier*gamepad1.right_stick_x);

            if (gamepad1.dpad_up) {
                OutPower = OutPower + 0.1;
            }

            if (gamepad1.dpad_down) {
                OutPower = OutPower - 0.1;
            }

            if (gamepad1.dpad_right) {
                BoostPower = BoostPower + 0.1;
            }

            if (gamepad1.dpad_left) {
                BoostPower = BoostPower - 0.1;
            }

            Boost.setPower(gamepad1.right_trigger*BoostPower);
            Out.setPower(gamepad1.left_trigger * OutPower);

            if (gamepad1.right_bumper && !lBumperStateCheck) {
                if (lI.getPower() > 0) {
                    lI.setPower(0);
                    rI.setPower(0);
                }else{
                    lI.setPower(1);
                    rI.setPower(-1);
                }

            }

            telemetry.addData("r", teleOpController.getHeading());
            telemetry.update();
            rBumperStateCheck = gamepad1.right_bumper;
            lBumperStateCheck = gamepad1.left_bumper;
        }
    }
}
