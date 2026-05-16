package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.shprobotics.pestocore.processing.MotorCortex;

@TeleOp(name = "Drive")
public class Drive extends BaseRobot {
    DcMotor lI;
    DcMotor rI;

    DcMotor lO;
    DcMotor rO;

//    AprilTagProcessor AT;

//
//    double turnRatio = 7.466666667; //time over rotation angle ratio
//    double strafeRatio = 1; //time over distance ratio
//    double moveRatio = 0.5; //time over distance ratio
//    double disFromAT = 30; //inches in depth from April Tag
//    double rotationSpeedMultiplier = 1; //multiplier on rotation speed
    boolean rBumperStateCheck = false;
    boolean lBumperStateCheck = false;


    @Override
    public void runOpMode() {

        super.runOpMode();
        rI = (DcMotor) hardwareMap.get("intakeRight");
        lI = (DcMotor) hardwareMap.get("intakeLeft");

        rO = (DcMotor) hardwareMap.get("shooterRight");
        lO = (DcMotor) hardwareMap.get("shooterLeft");

    waitForStart();

        while (opModeIsActive() && !isStopRequested()) {
                MotorCortex.update();
                gamepadInterface1.update();

                teleOpController.driveFieldCentric(-gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x);

                if (gamepad1.b) {
                    teleOpController.resetIMU(0);
                }

                telemetry.addData("imu", teleOpController.getHeading());
                telemetry.update();


                lI.setPower(-gamepad1.left_trigger);
                rI.setPower(gamepad1.left_trigger);

                lO.setPower(-gamepad1.right_trigger);
                rO.setPower(gamepad1.right_trigger);

                if (gamepad1.left_bumper){
                    lI.setPower(1);
                    rI.setPower(-1);
                }

                if (gamepad1.right_bumper){
                    lO.setPower(1);
                    rO.setPower(-1);
                }

                rBumperStateCheck = gamepad1.right_bumper;
                lBumperStateCheck = gamepad1.left_bumper;
            }
        }
    }
