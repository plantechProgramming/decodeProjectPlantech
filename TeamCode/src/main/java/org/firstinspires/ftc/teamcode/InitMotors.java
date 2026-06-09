package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.CameraName;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class InitMotors {
    HardwareMap hardwareMap;

    public static CRServo SL,SR;
    public static DcMotorEx FL, FR, BL, BR,inBetweenMotor, shootMotor, shootMotorOp,intakeMotor; //odometry is for testing purposes
    public static Telemetry dashboardTelemetry;
    public static GoBildaPinpointDriver odometry;
    public InitMotors(HardwareMap hardwareMap){
        this.hardwareMap = hardwareMap;
    }
    public void initDriveTrain(){
        FL = hardwareMap.get(DcMotorEx.class, "FL");
        FL.setDirection(DcMotorEx.Direction.REVERSE);
        FL.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        FL.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        FL.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        FR = hardwareMap.get(DcMotorEx.class, "FR");
        FR.setDirection(DcMotorEx.Direction.FORWARD);
        FR.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        FR.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        FR.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER );

        BL = hardwareMap.get(DcMotorEx.class, "BL");
        BL.setDirection(DcMotorEx.Direction.REVERSE);
        BL.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        BL.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        BL.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        BR = hardwareMap.get(DcMotorEx.class, "BR");
        BR.setDirection(DcMotorEx.Direction.FORWARD);
        BR.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        BR.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        BR.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
    }
    
    public void initIntake(){
        intakeMotor = hardwareMap.get(DcMotorEx.class,"Intake");
        intakeMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        intakeMotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        intakeMotor.setDirection(DcMotorSimple.Direction.REVERSE);
    }
    
    public void initInBetween(){
        SL = hardwareMap.get(CRServo.class,"SIBR");
        SR = hardwareMap.get(CRServo.class,"SIBL");

        inBetweenMotor = hardwareMap.get(DcMotorEx.class, "inbetween");
        inBetweenMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        inBetweenMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        inBetweenMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        odometry = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");
        odometry.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        odometry.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.REVERSED, GoBildaPinpointDriver.EncoderDirection.REVERSED);
        odometry.setOffsets(-155,-90, DistanceUnit.MM); //x = -155, y= -90
    }
    
    public void initShooter(){
        shootMotor = hardwareMap.get(DcMotorEx.class, "shooter");
        shootMotor.setDirection(DcMotorSimple.Direction.FORWARD);
//        shootMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        shootMotor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);

        shootMotorOp = hardwareMap.get(DcMotorEx.class, "shooter2");
        shootMotorOp.setDirection(DcMotorSimple.Direction.FORWARD);
//        shootMotorOp.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        shootMotorOp.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
    }
    
    public void initPinpiont(){
        odometry = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");
        odometry.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        odometry.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.REVERSED, GoBildaPinpointDriver.EncoderDirection.REVERSED);
        odometry.setOffsets(-155,-90, DistanceUnit.MM); //x = -155, y= -90
        odometry.resetPosAndIMU();
    }
    
    public void initDashboard(){
        FtcDashboard dashboard = FtcDashboard.getInstance();
        dashboardTelemetry = dashboard.getTelemetry();
    }
}
