package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.shprobotics.pestocore.hardware.CortexLinkedMotor;
import com.shprobotics.pestocore.processing.MotorCortex;

public class IntakeOuttakeSubsystem {
    private final CortexLinkedMotor theGoatorRight;
    //private final CortexLinkedMotor theGoatorLeft;
    private final CortexLinkedMotor shooter;
    private IntakeOuttakeState state;

    public enum IntakeOuttakeState{
        REJECT,
        NEUTRAL,
        PREREV,
        REV,
        INTAKE,
        OUTTAKE
    }

    public IntakeOuttakeSubsystem(){
        theGoatorRight = MotorCortex.getMotor("theGoatorRight");
        theGoatorRight.setDirection(DcMotorSimple.Direction.REVERSE);
        //theGoatorLeft = MotorCortex.getMotor("theGoatorLeft");
        shooter = MotorCortex.getMotor("shooter");
        state = IntakeOuttakeState.NEUTRAL;
    }

    public void setState(IntakeOuttakeState state){this.state = state;}

    public void update(){
        if (state == IntakeOuttakeState.INTAKE){
            theGoatorRight.setPowerResult(1);
           // theGoatorLeft.setPowerResult(1);
            shooter.setPowerResult(0);
        }
        else if (state == IntakeOuttakeState.REJECT){
            theGoatorRight.setPowerResult(-1);
           // theGoatorLeft.setPowerResult(-1);
            shooter.setPowerResult(-1);
        }
        else if (state == IntakeOuttakeState.PREREV){
            theGoatorRight.setPowerResult(0);
           // theGoatorLeft.setPowerResult(0);
            shooter.setPowerResult(0.5);
        }
        else if (state == IntakeOuttakeState.REV){
            theGoatorRight.setPowerResult(0);
           // theGoatorLeft.setPowerResult(0);
            shooter.setPowerResult(1);
        }
        else if (state == IntakeOuttakeState.OUTTAKE){
            theGoatorRight.setPowerResult(1);
            //theGoatorLeft.setPowerResult(1);
            shooter.setPowerResult(1);
        }
        else{
            theGoatorRight.setPowerResult(0);
          //  theGoatorLeft.setPowerResult(0);
            shooter.setPowerResult(0);
        }
    }
}
