package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;

@TeleOp
public class KEVIN extends LinearOpMode {

    DcMotor fL;
    DcMotor fR;
    DcMotor bL;
    DcMotor bR;

    DcMotor lI;
    DcMotor rI;


    DcMotor Out;

    DcMotor Boost;

    public void runOpMode(){
        fL = hardwareMap.get(DcMotor.class, "frontLeft");
        fR = hardwareMap.get(DcMotor.class, "frontRight");
        fR.setDirection(DcMotorSimple.Direction.REVERSE);
        bL = hardwareMap.get(DcMotor.class, "backLeft");
        bR = hardwareMap.get(DcMotor.class, "backRight");
        bR.setDirection(DcMotorSimple.Direction.REVERSE);

        lI = hardwareMap.get(DcMotor.class, "left");
        lI.setDirection(DcMotorSimple.Direction.REVERSE);
        rI = hardwareMap.get(DcMotor.class, "right");
        rI.setDirection(DcMotorSimple.Direction.REVERSE);

        Out = hardwareMap.get(DcMotor.class, "out");
        Boost = hardwareMap.get(DcMotor.class, "boost");
        Boost.setDirection(DcMotorSimple.Direction.REVERSE);

        waitForStart();
        while(opModeIsActive()){
                drive(gamepad1);
            }
        }

        public void drive(Gamepad g1){
            lI.setPower(gamepad1.right_trigger);
            rI.setPower(-gamepad1.right_trigger);

            Out.setPower(gamepad1.left_trigger);
            Boost.setPower(gamepad1.left_trigger);

            fL.setPower(-g1.left_stick_y + g1.left_stick_x  + g1.right_stick_x);
            fR.setPower(-g1.left_stick_y  - g1.left_stick_x - g1.right_stick_x);
            bL.setPower(-g1.left_stick_y  - g1.left_stick_x + g1.right_stick_x);
            bR.setPower(-g1.left_stick_y  + g1.left_stick_x - g1.right_stick_x);




        }

}
