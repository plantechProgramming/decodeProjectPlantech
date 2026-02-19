package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.bylazar.gamepad.Gamepad;
import com.bylazar.gamepad.GamepadManager;
import com.bylazar.gamepad.PanelsGamepad;
import com.bylazar.panels.Panels;
import com.bylazar.telemetry.PanelsTelemetry;
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
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.CameraName;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.teleOp.actions.DriveTrain;
import org.firstinspires.ftc.teamcode.teleOp.actions.*;


public abstract class OpMode extends LinearOpMode {

//    protected CRServo ;
//    protected Servo roni2_intake,IntakeR;
    protected CRServo intakeIBL,intakeIBR,shooterIBL,shooterIBR;
//    protected NormalizedColorSensor colorSensor;

    protected CameraName camera;
    protected DcMotorEx DriveFrontLeft, DriveFrontRight, DriveBackLeft, DriveBackRight,turretMotor, EH, EA,SU,SD, shootMotor, shootMotorOp,intakeMotor; //odometry is for testing purposes
    protected ElapsedTime runtime = new ElapsedTime();
    public boolean liftFlag = false;
    protected Telemetry dashboardTelemetry;
    protected GoBildaPinpointDriver odometry;
    protected IMU Imu;
    public FtcDashboard dashboard;
    public boolean pushed = false;
    void initialize() {

        DriveFrontLeft = hardwareMap.get(DcMotorEx.class, "FL");
        DriveFrontLeft.setDirection(DcMotorEx.Direction.REVERSE);
        DriveFrontLeft.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        DriveFrontLeft.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        DriveFrontLeft.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        DriveFrontRight = hardwareMap.get(DcMotorEx.class, "FR");
        DriveFrontRight.setDirection(DcMotorEx.Direction.FORWARD);
        DriveFrontRight.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        DriveFrontRight.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        DriveFrontRight.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        DriveBackLeft = hardwareMap.get(DcMotorEx.class, "BL");
        DriveBackLeft.setDirection(DcMotorEx.Direction.REVERSE);
        DriveBackLeft.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        DriveBackLeft.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        DriveBackLeft.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        DriveBackRight = hardwareMap.get(DcMotorEx.class, "BR");
        DriveBackRight.setDirection(DcMotorEx.Direction.FORWARD);
        DriveBackRight.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        DriveBackRight.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        DriveBackRight.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        //servos connected funny
        intakeIBR = hardwareMap.get(CRServo.class,"IBL");
        intakeIBL = hardwareMap.get(CRServo.class,"IBR");
        shooterIBL = hardwareMap.get(CRServo.class,"SIBR");
        shooterIBR = hardwareMap.get(CRServo.class,"SIBL");

//
        intakeMotor = hardwareMap.get(DcMotorEx.class,"Intake");
        intakeMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        intakeMotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
//        colorSensor = hardwareMap.get(NormalizedColorSensor.class, "color_sensor");

        Imu = hardwareMap.get(IMU.class, "imu");
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.RIGHT,
                RevHubOrientationOnRobot.UsbFacingDirection.UP));
        Imu.initialize(parameters);
        Imu.resetYaw();

        shootMotor = hardwareMap.get(DcMotorEx.class, "shooter");
        shootMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        shootMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        shootMotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        shootMotorOp = hardwareMap.get(DcMotorEx.class, "shooter2");
        shootMotorOp.setDirection(DcMotorSimple.Direction.FORWARD);
        shootMotorOp.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        shootMotorOp.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        turretMotor = hardwareMap.get(DcMotorEx.class, "turret");
        turretMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        turretMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        turretMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        odometry = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");
        odometry.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        odometry.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.REVERSED, GoBildaPinpointDriver.EncoderDirection.REVERSED);
        odometry.setOffsets(-155,-90, DistanceUnit.MM); //x = -155, y= -90
//        odometry.resetPosAndIMU();

        // until we find the fucking camera we can't scan it and add it to robot config :(((
    //    camera = hardwareMap.get(CameraName.class,"webcam");

        FtcDashboard dashboard = FtcDashboard.getInstance();
        dashboardTelemetry = dashboard.getTelemetry();
    }

    public DcMotorEx initMotor(String name, boolean encoder, boolean reversed){
        DcMotorEx motor = hardwareMap.get(DcMotorEx.class,name);

        if(encoder) {motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);}
        else {motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);}

        if(reversed){motor.setDirection(DcMotorEx.Direction.REVERSE);}
        else{motor.setDirection(DcMotorSimple.Direction.FORWARD);}

        return motor;
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
    public void push(CRServo pushT){
        double Power = 0.5;
        if (pushed == false)
            pushT.setPower(Power);
        else
            pushT.setPower(-Power);
    }

    protected void postInit() {

    }
    protected abstract void run();

    protected abstract void end();
}

