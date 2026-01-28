package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@Autonomous
public class KEVIN_AUTO_BL extends LinearOpMode {

    DcMotor fL;
    DcMotor fR;
    DcMotor bL;
    DcMotor bR;

    DcMotor lI;
    DcMotor rI;


    DcMotor Out;
    DcMotor Boost;


    //
    //BASIC METHODS
    //

    public void move_bot(double left_stick_y, double left_stick_x, double right_stick_x, int time){
        fL.setPower(left_stick_y + left_stick_x + right_stick_x);
        fR.setPower(left_stick_y - left_stick_x - right_stick_x);
        bL.setPower(left_stick_y - left_stick_x + right_stick_x);
        bR.setPower(left_stick_y + left_stick_x - right_stick_x);
        sleep(time);
    }

    public void bot_stop(int time){
        move_bot(0,0,0, 0);
        set_intake(0.3, 0);
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



    //
    //STEP METHODS
    //

    public void FIREEEEE (){
        //FIRE!!! - fires, ends in same position as starts; but starts forwards & ends facing target
        move_bot(0, 0, -0.5, 1660-80);
        bot_stop(250);
        set_outake(1, 1, 250);
        set_intake(1, 50);
        move_bot(-1, 0, 0, 100);
        move_bot(0,0,0, 2000);
        sleep(2000);
        move_bot(1, 0, 0, 50);
        sleep(250);
        bot_stop(250);
    }

    public void GO_BACK_TO_COLLECT_NEXT_BALL (){
        //GO BACK TO COLLECT NEXT BALL - goes back to line of balls, turns to face them, does not collect
        move_bot(0, 0, -0.5, 2290);
        bot_stop(250);
        move_bot(0.5, 0, 0, 1320);
        bot_stop(250);
        move_bot(0, 0, -0.5, 2016);
        bot_stop(250);
    }

    public void GRAB_NEXT_BALL (int num){
        //GRAB NEXT BALL - moves forward to collect ball and back, ends in same position as starts
        move_bot(0.5, 0, 0, 1050+2*num);
        bot_stop(10);
        set_intake(1, 50);
        bot_stop(50);
        move_bot(-0.5, 0, 0, 1050+2*num);
        bot_stop(250);
    }

    public void GO_BACK_TO_SCORING_POS (){
        //GO BACK TO SCORING POS - goes back to shooting position, ends facing forwards
        move_bot(0, 0, -0.5, 2016);
        bot_stop(250);
        move_bot(0.5, 0, 0, 1320);
        bot_stop(250);
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

//        long start = System.nanoTime();

        waitForStart();

        while (opModeIsActive()) {

            //FIRE PRE-LOADED BALL
                //MOVE INTO POS
                move_bot(1, 0, 0, 1000);
                bot_stop(250);

                FIREEEEE();
                GO_BACK_TO_COLLECT_NEXT_BALL();

            //GRAB, COLLECT, FIRE --> FIRST BALL
                GRAB_NEXT_BALL(15);
                GO_BACK_TO_SCORING_POS();
                FIREEEEE();
                GO_BACK_TO_COLLECT_NEXT_BALL();


            //GRAB, COLLECT, FIRE --> SECOND BALL
                GRAB_NEXT_BALL(30);
                GO_BACK_TO_SCORING_POS();
                FIREEEEE();
                GO_BACK_TO_COLLECT_NEXT_BALL();

            //GRAB, COLLECT, FIRE --> SECOND BALL
                GRAB_NEXT_BALL(45);
                GO_BACK_TO_SCORING_POS();
                FIREEEEE();

            //GO BACK TO START
                move_bot(0, 0, -0.5, 1008);
                bot_stop(250);
                move_bot(1, 0, 0, 900);
                bot_stop(250);
                move_bot(0, 0, -0.5, 1344);
                bot_stop(0);
                set_intake(0, 0);
                sleep(1000000000);

        }
    }
}