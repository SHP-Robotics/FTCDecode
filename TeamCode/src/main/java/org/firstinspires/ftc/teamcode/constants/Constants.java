package org.firstinspires.ftc.teamcode.constants;

import com.pedropathing.control.FilteredPIDFCoefficients;
import com.pedropathing.control.PIDFCoefficients;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.ftc.FollowerBuilder;
import com.pedropathing.ftc.drivetrains.MecanumConstants;
import com.pedropathing.ftc.localization.Encoder;
import com.pedropathing.ftc.localization.constants.ThreeWheelConstants;
import com.pedropathing.paths.PathConstraints;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Constants {
    public static FollowerConstants followerConstants;

    public static PathConstraints pathConstraints = new PathConstraints(0.99, 100);

    public static MecanumConstants driveConstants = new MecanumConstants()
            .maxPower(1.0)
            .rightFrontMotorName("fr")
            .rightRearMotorName("br")
            .leftRearMotorName("bl")
            .leftFrontMotorName("fl")
            .leftFrontMotorDirection(DcMotorSimple.Direction.REVERSE)
            .leftRearMotorDirection(DcMotorSimple.Direction.REVERSE)
            .rightFrontMotorDirection(DcMotorSimple.Direction.FORWARD)
            .rightRearMotorDirection(DcMotorSimple.Direction.FORWARD);

    public static ThreeWheelConstants localizerConstants = new ThreeWheelConstants()
            .forwardTicksToInches(0.001978956176) // TODO: tune
            .strafeTicksToInches(0.001978956176) // TODO: tune
            .turnTicksToInches(0.001978956176) // TODO: tune
            .leftPodY(5.05) // TODO: tune
            .rightPodY(-5.05) // TODO: tune
            .strafePodX(-1.565) // TODO: tune
            .leftEncoder_HardwareMapName("fl")
            .rightEncoder_HardwareMapName("fr")
            .strafeEncoder_HardwareMapName("bl")
            .leftEncoderDirection(Encoder.FORWARD)
            .rightEncoderDirection(Encoder.FORWARD)
            .strafeEncoderDirection(Encoder.REVERSE);

    public static Follower createFollower(HardwareMap hardwareMap) {
        followerConstants = new FollowerConstants()
                .useSecondaryHeadingPIDF(true)
                .useSecondaryDrivePIDF(true)
                .useSecondaryTranslationalPIDF(true)

                .headingPIDFSwitch(0.2)
                .headingPIDFCoefficients(new PIDFCoefficients(0.8, 0, 0, 0))
                .secondaryHeadingPIDFCoefficients(new PIDFCoefficients(2.0, 0, 0, 0))

                .drivePIDFSwitch(2)
                .drivePIDFCoefficients(new FilteredPIDFCoefficients(0.1, 0, 0.01, 0.6, 0.0))
                .secondaryDrivePIDFCoefficients(new FilteredPIDFCoefficients(0.0, 0, 0.0, 0.0, 0.0))

                .mass(7.7); // 7.7kg == 17lbs

        followerConstants.setForwardZeroPowerAcceleration(-40.3);
        followerConstants.setLateralZeroPowerAcceleration(-85);

        driveConstants.setXVelocity(79.8);
        driveConstants.setYVelocity(64);

        return new FollowerBuilder(followerConstants, hardwareMap)
                .threeWheelLocalizer(localizerConstants)
                .pathConstraints(pathConstraints)
                .mecanumDrivetrain(driveConstants)
                .build();
    }
}
