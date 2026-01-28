package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import java.util.Arrays;
import java.util.List;

@Autonomous
public class TestStrafe extends LinearOpMode{
    DcMotor fL;
    DcMotor fR;
    DcMotor bL;
    DcMotor bR;
    DcMotor lI;
    DcMotor rI;

    DcMotor Out;
    DcMotor Boost;
    Servo Push;

    public void move_bot(double left_stick_y, double left_stick_x, double right_stick_x, int time){
        fL.setPower(-left_stick_y + left_stick_x + right_stick_x);
        fR.setPower(-left_stick_y - left_stick_x - right_stick_x);
        bL.setPower(-left_stick_y - left_stick_x + right_stick_x);
        bR.setPower(-left_stick_y + left_stick_x - right_stick_x);
        sleep(time);
    }

    public void bot_stop(int time){
        move_bot(0,0,0, 0);
        set_intake(0, 0);
        set_outake(0, 0, time);
    }

    public void set_intake(double right_trigger, int time) {
        lI.setPower(right_trigger);
        rI.setPower(-right_trigger);
        sleep(time);
    }

    public void set_outake(double left_trigger, double left_bumper, int time) {
        Out.setPower(0.8*left_trigger);
        Boost.setPower(left_bumper);
        sleep(time);
    }



    public void runOpMode() {
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

        long start = System.nanoTime();

        waitForStart();

        while (opModeIsActive()) {

            double right_trigger = 0;
            double left_trigger = 0;
            double left_stick_x = 0;
            double left_stick_y = 0;
            double right_stick_x = 0;

            //TEST LEFT & RIGHT LENGTHS
            List<Integer> move_times = Arrays.asList(500, 750, 1000, 1250, 1500, 2000, 2500);

            for (int t : move_times) {
                move_bot(0, 1, 0, t);
                bot_stop(7000);
                move_bot(0, -1, 0, t);
                bot_stop(5000);
            }

            sleep(15000);

            for (int t : move_times) {
                move_bot(-1, -1, 0, t);
                bot_stop(7000);
                move_bot(1, 1, 0, t);
                bot_stop(5000);
            }
        }
    }
}
