package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.VoltageSensor;

@Autonomous
public class GalaxyAuto extends LinearOpMode {

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

    public double getBatteryVoltage() {
        double result = Double.POSITIVE_INFINITY;
        for (VoltageSensor sensor : hardwareMap.voltageSensor) {
            double voltage = sensor.getVoltage();
            if (voltage > 0) {
                result = Math.min(result, voltage);
            }
        }
        return result;
    }

    public void turn(double angle, int ccw_is_neg, double BatteryOffset, double Ratio){
        int direction = ccw_is_neg;
        move_bot(0,0, direction, ((int)(BatteryOffset*Ratio*angle*0.5)));
    }

    public void forward(double tile_dist, double BatteryOffset, double Ratio) {
        move_bot(1, 0, 0, ((int) (BatteryOffset * Ratio * tile_dist)));
    }

    public void backward(double tile_dist, double BatteryOffset, double Ratio) {
        move_bot(-1, 0, 0, ((int) (BatteryOffset * Ratio * tile_dist)));
    }

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
        Out.setPower(left_trigger);
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

//      long start = System.nanoTime();


        waitForStart();

        if (opModeIsActive()) {

            //INITIALIZE STUFF
            double random_a = 112;
            double random_b = 15;
            double TimeToAngleRatio_ccw = random_a/random_b;
            double TimeToForwardRatio = 320;
            double BatterySet = 12.41;
            double CurrentVoltage = getBatteryVoltage();
            double BatteryOffset = BatterySet/CurrentVoltage;


            set_outake(1, 1, 5000);
            set_intake(1, 5000);

            backward(1, BatteryOffset, TimeToForwardRatio);
            forward(0.5, BatteryOffset, TimeToForwardRatio);
            bot_stop(10);

            backward(1, BatteryOffset, TimeToForwardRatio);
            forward(0.5, BatteryOffset, TimeToForwardRatio);
            bot_stop(10);

            turn(110, 1, BatteryOffset, TimeToAngleRatio_ccw);
            forward(1.5, BatteryOffset, TimeToForwardRatio);
            bot_stop(10);
        }
    }
}