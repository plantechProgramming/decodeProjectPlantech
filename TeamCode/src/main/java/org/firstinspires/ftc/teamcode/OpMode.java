package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcontroller.external.samples.SensorColor;
import org.firstinspires.ftc.teamcode.teleOp.actions.DriveTrain;


public abstract class OpMode extends LinearOpMode {

//    protected CRServo ;
    protected Servo roni2_intake,intake_center_angle;
    protected CRServo IntakeL,IntakeR;
    protected NormalizedColorSensor colorSensor;
    protected DcMotorEx DriveFrontLeft, DriveFrontRight, DriveBackLeft, DriveBackRight, EH, EA,SU,SD;
    protected ElapsedTime runtime = new ElapsedTime();
    public boolean liftFlag = false;


    protected IMU Imu;

    FtcDashboard dashboard;

    void initialize() {
//        DriveFrontLeft = hardwareMap.get(DcMotorEx.class, "FL");
//        DriveFrontLeft.setDirection(DcMotorEx.Direction.REVERSE);
//        DriveFrontLeft.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
//        DriveFrontLeft.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
//        DriveFrontLeft.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
//
//        DriveFrontRight = hardwareMap.get(DcMotorEx.class, "FR");
//        DriveFrontRight.setDirection(DcMotorEx.Direction.FORWARD);
//        DriveFrontRight.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
//        DriveFrontRight.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
//        DriveFrontRight.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
//
//        DriveBackLeft = hardwareMap.get(DcMotorEx.class, "BL");
//        DriveBackLeft.setDirection(DcMotorEx.Direction.REVERSE);
//        DriveBackLeft.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
//        DriveBackLeft.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
//        DriveBackLeft.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
//
//        DriveBackRight = hardwareMap.get(DcMotorEx.class, "BR");
//        DriveBackRight.setDirection(DcMotorEx.Direction.FORWARD);
//        DriveBackRight.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
//        DriveBackRight.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
//        DriveBackRight.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
//
//        EA = hardwareMap.get(DcMotorEx.class, "EA");
//        EA.setDirection(DcMotorEx.Direction.REVERSE);
//        EA.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
//        EA.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
//        EA.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
//
//        EH = hardwareMap.get(DcMotorEx.class, "EH");
//        EH.setDirection(DcMotorEx.Direction.FORWARD);
//        EH.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
//        EH.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
//        EH.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        SU = hardwareMap.get(DcMotorEx.class, "ShooterU");
        SU.setDirection(DcMotorEx.Direction.FORWARD);
        SU.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        SU.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        SU.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        SD = hardwareMap.get(DcMotorEx.class, "ShooterD");
        SD.setDirection(DcMotorEx.Direction.FORWARD);
        SD.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        SD.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        SD.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        intake_center_angle = hardwareMap.get(Servo.class,"intA");
        intake_center_angle.setPosition(0.7);
        colorSensor = hardwareMap.get(NormalizedColorSensor.class, "color_sensor");
        IntakeR = hardwareMap.get(CRServo.class,"IntakeR");
        IntakeL = hardwareMap.get(CRServo.class,"IntakeL");


        Imu = hardwareMap.get(IMU.class, "imu");
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.RIGHT,
                RevHubOrientationOnRobot.UsbFacingDirection.UP));
        Imu.initialize(parameters);
        Imu.resetYaw();

    }

    @Override
    public void runOpMode() throws InterruptedException  {
        initialize();
        waitForStart();
        postInit();

        dashboard = FtcDashboard.getInstance();
        if (opModeIsActive()) {
            run();
        }

        end();
    }

    protected void postInit() {

    }
    protected abstract void run();
//    @Override
//    protected abstract void init();
//    @Override
//    protected abstract void loop();

    protected abstract void end();
}

