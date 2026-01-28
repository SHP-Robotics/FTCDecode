package org.firstinspires.ftc.teamcode;

import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.BRAKE;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.shprobotics.pestocore.processing.MotorCortex;

@TeleOp(name = "TestB_F")
public class TestB_F extends BaseRobot {
    DcMotor lI;
    DcMotor rI;

    DcMotor Out;
    DcMotor Boost;

//    AprilTagProcessor AT;


    double turnRatio = 7.466666666666666666666666666666666666666666666666666666666666666666666666667; //time over rotation angle ratio
    double strafeRatio = 1; //time over distance ratio
    double moveRatio = 0.5; //time over distance ratio
    double disFromAT = 30; //inches in depth from April Tag
    double rotationSpeedMultiplier = 1; //multiplier on rotation speed
    boolean rBumperStateCheck = false;
    boolean lBumperStateCheck = false;

    public void turn_180 (double turnRatio) {
        // Turn 180 so output is facing target
        teleOpController.driveRobotCentric(0, 0, 0.5);
        sleep(1344);
        teleOpController.driveRobotCentric(0,0,0);
    }

    @Override
    public void runOpMode() {

        lI = hardwareMap.get(DcMotor.class, "left");
        lI.setDirection(DcMotorSimple.Direction.REVERSE);
        rI = hardwareMap.get(DcMotor.class, "right");
        rI.setDirection(DcMotorSimple.Direction.REVERSE);

        Out = hardwareMap.get(DcMotor.class, "out");
        Boost = hardwareMap.get(DcMotor.class, "boost");
        Boost.setDirection(DcMotorSimple.Direction.REVERSE);

        // BRAKE is only active when setPower = 0.0
        Out.setZeroPowerBehavior(BRAKE);
        Boost.setZeroPowerBehavior(BRAKE);

        super.runOpMode();

        waitForStart();

        while (opModeIsActive() && !isStopRequested()) {

            MotorCortex.update();
            gamepadInterface1.update();

            teleOpController.driveFieldCentric(-gamepad1.left_stick_y, gamepad1.left_stick_x, rotationSpeedMultiplier*gamepad1.right_stick_x);

            if (gamepad1.b) {
                teleOpController.resetIMU(0);
            }

            if (gamepad1.right_trigger < 0.3) {
                lI.setPower(0.3);
                rI.setPower(-0.3);
            } else {
                lI.setPower(gamepad1.right_trigger);
                rI.setPower(-gamepad1.right_trigger);
            }


            if (gamepad1.left_trigger > 0.05) {
                Out.setPower(0.7);
            } else {
                Out.setPower(0.0);
            }

            if (gamepad1.left_trigger > 0.95) {
                Boost.setPower(0.6);
            } else {
                Boost.setPower(0.0);
            }

            if (gamepad1.right_bumper && !rBumperStateCheck) {
                if (rotationSpeedMultiplier == 1) {
                    rotationSpeedMultiplier = 0.5;
                }else{
                    rotationSpeedMultiplier = 1;
                }
            }

//                if (gamepad1.dpad_left) {
//                    telemetry.addLine("You drive like you used to have a brain eating ameba, but it died of hunger.");
//                    telemetry.addLine("- Max");
//                    telemetry.update();
//                }
//
//                if (gamepad1.x) {
//                    telemetry.addLine("You drive like you're playing Mario Kart 2.");
//                    telemetry.addLine("- Max");
//                    telemetry.update();
//                }
//
//                if (gamepad1.dpad_right) {
//                    telemetry.addLine("You drive like the question isn't which drug you are on, it's which one you aren't on.");
//                    telemetry.addLine("- Max");
//                    telemetry.update();
//                }
//
//                if (gamepad1.y) {
//                    telemetry.addLine("If driving robots poorly was a crime, the electric chair wouldn't be enough for you.");
//                    telemetry.addLine("- Max");
//                    telemetry.update();
//                }
//
//                if (gamepad1.left_bumper && !lBumperStateCheck) {
//                    telemetry.addLine("Your driving singlehandedly puts humans back 10 years on the evolutionary scale.");
//                    telemetry.addLine("- Max");
//                    telemetry.update();
//                }

            rBumperStateCheck = gamepad1.right_bumper;
            lBumperStateCheck = gamepad1.left_bumper;
        }
    }
}
//}
