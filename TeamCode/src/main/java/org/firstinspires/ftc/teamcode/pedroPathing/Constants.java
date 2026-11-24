package org.firstinspires.ftc.teamcode.pedroPathing;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.control.PIDFCoefficients;
import com.pedropathing.control.FilteredPIDFCoefficients;
import com.pedropathing.control.PredictiveBrakingCoefficients;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.ftc.FollowerBuilder;
import com.pedropathing.ftc.drivetrains.MecanumConstants;
import com.pedropathing.ftc.localization.constants.PinpointConstants;
import com.pedropathing.paths.PathConstraints;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

@Configurable
public class Constants {
    public static MecanumConstants driveConstants = new MecanumConstants()
            .leftFrontMotorName("lfd")
            .leftRearMotorName("lbd")
            .rightFrontMotorName("rfd")
            .rightRearMotorName("rbd")
            .leftFrontMotorDirection(DcMotorSimple.Direction.REVERSE)
            .leftRearMotorDirection(DcMotorSimple.Direction.REVERSE)
            .rightFrontMotorDirection(DcMotorSimple.Direction.FORWARD)
            .rightRearMotorDirection(DcMotorSimple.Direction.FORWARD)

            .xVelocity(89.37217152512918)
            .yVelocity(70.21966072142592);

    public static FollowerConstants followerConstants = new FollowerConstants()
            .mass(10.1)
            .forwardZeroPowerAcceleration(-34.172566941198305)
            .lateralZeroPowerAcceleration(-73.08409644871489)
            .predictiveBrakingCoefficients(new PredictiveBrakingCoefficients(0.1, 0.04, 0.0016))
            .translationalPIDFCoefficients(new PIDFCoefficients(
                    0.11,
                    0,
                    0.015,
                    0.01
            ))
//            .translationalPIDFCoefficients(new PIDFCoefficients(
//                    0.08,dw3
//                    0,
//                    0.01,
//                    0.04
//            ))
            .translationalPIDFSwitch(5)
            .secondaryTranslationalPIDFCoefficients(new PIDFCoefficients(
                    0.06,
                    0,
                    0.004,                     //0.004,
                    0.04                //0.02
            ))
//            .secondaryTranslationalPIDFCoefficients(new PIDFCoefficients(
//                    0.4,   //0.4
//                    0,
//                    0.005,       //0.005
//                    0.0006
//            ))
            .useSecondaryTranslationalPIDF(false)
//            .headingPIDFCoefficients(new PIDFCoefficients(
//                    0.64,
//                    0,
//                    0.022,
//                    0.02
//            ))
            .headingPIDFCoefficients(new PIDFCoefficients(
                    0.75,
                    0,
                    0.06,
                    0.02
            ))
            .secondaryHeadingPIDFCoefficients(new PIDFCoefficients(
                    2, //2.5
                    0,
                    0.1,   //0.1
                    0.002 //0.0005
            ))
//            .secondaryHeadingPIDFCoefficients(new PIDFCoefficients(
//                    1, //2.5
//                    0,
//                    0.1,   //0.1
//                    0.02 //0.0005
//            ))
            .useSecondaryHeadingPIDF(false)
            .drivePIDFCoefficients(new FilteredPIDFCoefficients(
                    0.4,
                    0,
                    0.000000005,
                    0.6,
                    0.0009

            ))
            .useSecondaryDrivePIDF(false)
            .secondaryDrivePIDFCoefficients(new FilteredPIDFCoefficients(
                    0.02,
                    0,
                    0.000005,
                    0.6,
                    0.01
            ))

            .drivePIDFSwitch(15)
            .headingPIDFSwitch(0.2)
            .holdPointHeadingScaling(1)
            .holdPointTranslationalScaling(1)
            .turnHeadingErrorThreshold(0.005)
            .automaticHoldEnd(true)


            .centripetalScaling(0.0005);


    public static PinpointConstants localizerConstants = new PinpointConstants()
            .forwardPodY(-5.28231)
            .strafePodX(1.672886)
            .hardwareMapName("pinpoint")
            .encoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD)


            .forwardEncoderDirection(GoBildaPinpointDriver.EncoderDirection.REVERSED)
            .strafeEncoderDirection(GoBildaPinpointDriver.EncoderDirection.FORWARD);


    public static PathConstraints pathConstraints = new PathConstraints(
            0.995,
            1,
            1,
            1,
            100,
            1,
            10,
            1
    );

    public static Follower createFollower(HardwareMap hardwareMap) {
        return new FollowerBuilder(followerConstants, hardwareMap)

                .mecanumDrivetrain(driveConstants)
                .pinpointLocalizer(localizerConstants)
                .pathConstraints(pathConstraints)
                .build();
    }
}