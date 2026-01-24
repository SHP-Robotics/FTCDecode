package org.firstinspires.ftc.teamcode.teleops;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.shprobotics.pestocore.processing.MotorCortex;

import org.firstinspires.ftc.teamcode.PestoFTCConfig;
import org.firstinspires.ftc.teamcode.subsystems.BaseRobot;
import org.firstinspires.ftc.teamcode.subsystems.BlockerSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.HoodSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IntakeOuttakeSubsystem;

@Autonomous
public class Trust  extends BaseRobot {
    @Override
    public void runOpMode(){
        PestoFTCConfig.initializePinpoint = false;
        super.initialize();
        waitForStart();
        intakeOuttakeSubsystem.setState(IntakeOuttakeSubsystem.IntakeOuttakeState.PREREV);
        blockerSubsystem.setState(BlockerSubsystem.BlockerState.BLOCK);
        hoodSubsystem.setState(HoodSubsystem.HoodState.FAR);
        double start = System.nanoTime()/1E9;
        while(opModeIsActive() && !isStopRequested() && System.nanoTime()/1E9 - start < 15){
            if((System.nanoTime()/1E9) - start > 0.3 && (System.nanoTime()/1E9) - start < 2){
                intakeOuttakeSubsystem.setState(IntakeOuttakeSubsystem.IntakeOuttakeState.REV);
            }
            else if((System.nanoTime()/1E9) - start > 2){
                intakeOuttakeSubsystem.setState(IntakeOuttakeSubsystem.IntakeOuttakeState.OUTTAKE);
                blockerSubsystem.setState(BlockerSubsystem.BlockerState.NEUTRAL);
            }
            MotorCortex.update();
            intakeOuttakeSubsystem.update();
            blockerSubsystem.update();
            hoodSubsystem.update();
        }
    }


}

