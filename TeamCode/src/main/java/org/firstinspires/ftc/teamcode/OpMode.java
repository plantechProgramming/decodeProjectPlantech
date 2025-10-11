package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.bylazar.gamepad.Gamepad;
import com.bylazar.gamepad.GamepadManager;
import com.bylazar.gamepad.PanelsGamepad;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcontroller.external.samples.SensorColor;
import org.firstinspires.ftc.robotcore.external.hardware.camera.CameraName;
import org.firstinspires.ftc.teamcode.teleOp.actions.DriveTrain;


public abstract class OpMode extends LinearOpMode {

//    protected CRServo ;
    protected Servo roni2_intake,intake_center_angle;
    protected CRServo IntakeL,IntakeR;
    protected NormalizedColorSensor colorSensor;

    protected CameraName camera;
    protected DcMotorEx DriveFrontLeft, DriveFrontRight, DriveBackLeft, DriveBackRight, EH, EA,SU,SD, shootMotor, odometry, shootMotorOp; //odometry is for testing purposes
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

        shootMotor = hardwareMap.get(DcMotorEx.class, "shooter");
        shootMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        shootMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        shootMotor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);

        shootMotorOp = hardwareMap.get(DcMotorEx.class, "shooter2");
        shootMotorOp.setDirection(DcMotorSimple.Direction.FORWARD);
        shootMotorOp.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        shootMotorOp.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);

        odometry = hardwareMap.get(DcMotorEx.class, "ShooterD");
        odometry.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        odometry.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        camera = hardwareMap.get(CameraName.class,"webcam");
//        shooter.setDirection(DcMotorEx.Direction.FORWARD);
//        shooter.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
//        shooter.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
//        shooter.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
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

