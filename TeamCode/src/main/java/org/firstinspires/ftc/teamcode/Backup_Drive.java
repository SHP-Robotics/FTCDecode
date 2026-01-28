package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.shprobotics.pestocore.processing.MotorCortex;

@TeleOp(name = "Backup_Drive")
public class Backup_Drive extends BaseRobot {
    DcMotor lI;
    DcMotor rI;

    DcMotor Out;
    DcMotor Boost;

    double rotationSpeedMultiplier = 1; //multiplier on rotation speed
    boolean rBumperStateCheck = false;
    boolean lBumperStateCheck = false;

    public void turn_180 (double turnRatio) {
        // Turn 180 so output is facing target
        teleOpController.driveRobotCentric(0, 0, 0.5);
        sleep((int) Math.round(1344));
        teleOpController.driveRobotCentric(0,0,0);
        teleOpController.driveFieldCentric(-gamepad1.left_stick_y, gamepad1.left_stick_x, rotationSpeedMultiplier*gamepad1.right_stick_x);

    }
    public void fire_w_power (double powerOut, double powerBoost) {
//        set_outake(1, 1, 250);
//        set_intake(1, 50);
//        move_bot(1, 0, 0, 20);
//        bot_stop(50);
//        move_bot(-1, 0, 0, 20);
//        bot_stop(250);

        Out.setPower(powerOut);
        Boost.setPower(powerBoost);
        sleep(1000);
        lI.setPower(1);
        rI.setPower(-1);

        sleep(1000);

        teleOpController.driveRobotCentric(-1, 0, 0);
        sleep(400);
        teleOpController.driveRobotCentric(1, 0, 0);
        sleep(400);

        Out.setPower(0);
        Boost.setPower(0);
        lI.setPower(0);
        rI.setPower(0);
        teleOpController.driveFieldCentric(-gamepad1.left_stick_y, gamepad1.left_stick_x, rotationSpeedMultiplier*gamepad1.right_stick_x);
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


        waitForStart();

        while (opModeIsActive() && !isStopRequested()) {

            MotorCortex.update();
            gamepadInterface1.update();

            teleOpController.driveFieldCentric(-gamepad1.left_stick_y, gamepad1.left_stick_x, rotationSpeedMultiplier*gamepad1.right_stick_x);

            if (gamepad1.b) {
                    teleOpController.resetIMU(180);
                }

            lI.setPower(gamepad1.right_trigger);
            rI.setPower(-gamepad1.right_trigger);

            Out.setPower(gamepad1.left_trigger * 0.7);

            if (gamepad1.left_bumper && !lBumperStateCheck) {
                double i = Boost.getPower();
                i = (i + 0.7) % 1.4;
                Boost.setPower(i);
            }

            if (gamepad1.dpad_up) {
                fire_w_power(0.75, 0.75);
            }

            if (gamepad1.dpad_down) {
                turn_180(15);
            }

            if (gamepad1.right_bumper && !rBumperStateCheck) {
                if (rotationSpeedMultiplier == 1) {
                    rotationSpeedMultiplier = 0.5;
                }else{
                    rotationSpeedMultiplier = 1;
                }
            }

            if (gamepad1.dpad_left) {
                telemetry.addLine("You drive like you used to have a brain eating ameba, but it died of hunger.");
                telemetry.addLine("- Max");
                telemetry.update();
            }

            if (gamepad1.x) {
                telemetry.addLine("You drive like you're playing Mario Kart 2.");
                telemetry.addLine("- Max");
                telemetry.update();
            }

            if (gamepad1.dpad_right) {
                telemetry.addLine("You drive like the question isn't which drug you are on, it's which one you aren't on.");
                telemetry.addLine("- Max");
                telemetry.update();
            }

            if (gamepad1.y) {
                telemetry.addLine("If driving robots poorly was a crime, the electric chair wouldn't be enough for you.");
                telemetry.addLine("- Max");
                telemetry.update();
            }

            if (gamepad1.a) {
                telemetry.addLine("Your driving singlehandedly puts humans back 10 years on the evolutionary scale.");
                telemetry.addLine("- Max");
                telemetry.update();
            }

            telemetry.addData("r", teleOpController.getHeading());
            telemetry.update();
            rBumperStateCheck = gamepad1.right_bumper;
            lBumperStateCheck = gamepad1.left_bumper;
            teleOpController.driveFieldCentric(-gamepad1.left_stick_y, gamepad1.left_stick_x, rotationSpeedMultiplier*gamepad1.right_stick_x);

        }
    }
}
