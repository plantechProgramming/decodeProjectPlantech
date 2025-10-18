//package org.firstinspires.ftc.teamcode.auto.pedro.constants;
//
//import com.pedropathing.follower.Follower;
//import com.pedropathing.follower.FollowerConstants;
//import com.pedropathing.localization.Encoder;
//import com.pedropathing.localization.constants.TwoWheelConstants;
//import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
//import com.qualcomm.robotcore.hardware.DcMotorSimple;
//import com.qualcomm.robotcore.hardware.HardwareMap;
//import com.qualcomm.robotcore.hardware.PIDFCoefficients;
//
//public class Constants {
//
//    public static FollowerConstants followerConstants = new FollowerConstants().mass(10)
//            .forwardZeroPowerAcceleration(-39.9624986729912)
//            .lateralZeroPowerAcceleration(-60.78696124604437)
//            .useSecondaryTranslationalPIDF(false)
//            .useSecondaryHeadingPIDF(false)
//            .useSecondaryDrivePIDF(false)
//            .centripetalScaling(0.0007)
//            .translationalPIDFCoefficients(new PIDFCoefficients(0.2, 0, 0.01, 0))
//            .headingPIDFCoefficients(new PIDFCoefficients(2, 0, 0.1, 0))
//            .drivePIDFCoefficients(
//                    new FilteredPIDFCoefficients(0.01, 0, 0.000001, 0.1, 0)
//            );
//
//    public static MecanumConstants driveConstants = new MecanumConstants()
//            .leftFrontMotorName("FL")
//            .leftRearMotorName("BL")
//            .rightFrontMotorName("FR")
//            .rightRearMotorName("BR")
//            .leftFrontMotorDirection(DcMotorSimple.Direction.REVERSE)
//            .leftRearMotorDirection(DcMotorSimple.Direction.REVERSE)
//            .rightFrontMotorDirection(DcMotorSimple.Direction.FORWARD)
//            .rightRearMotorDirection(DcMotorSimple.Direction.FORWARD)
//            .xVelocity(58)
//            .yVelocity(46.85885628727077);
//
//    public static TwoWheelConstants localizerConstants = new TwoWheelConstants()
//            .forwardTicksToInches(0.003)
//            .strafeTicksToInches(0.00196721068372003)
//            .forwardPodY(-18.25)
//            .strafePodX(12.1)
//            .forwardEncoder_HardwareMapName("FR")
//            .strafeEncoder_HardwareMapName("BL")
//            .forwardEncoderDirection(Encoder.FORWARD)
//            .strafeEncoderDirection(Encoder.REVERSE)
//            .IMU_HardwareMapName("imu")
//            .IMU_Orientation(
//                    new RevHubOrientationOnRobot(
//                            RevHubOrientationOnRobot.LogoFacingDirection.UP,
//                            RevHubOrientationOnRobot.UsbFacingDirection.FORWARD
//                    )
//            );
//
//    public static PathConstraints pathConstraints = new PathConstraints(
//            0.995,
//            500,
//            1,
//            1
//    );
//
//    public static Follower createFollower(HardwareMap hardwareMap) {
//        return new FollowerBuilder(followerConstants, hardwareMap)
//                .mecanumDrivetrain(driveConstants)
//                .twoWheelLocalizer(localizerConstants)
//                .pathConstraints(pathConstraints)
//                .build();
//    }
//}

