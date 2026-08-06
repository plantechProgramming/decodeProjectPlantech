package org.firstinspires.ftc.teamcode.Misc;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class InitMotors {
    HardwareMap hardwareMap;

    public static CRServo SL,SR;
    public static DcMotorEx FL, FR, BL, BR; //odometry is for testing purposes
    public static DcMotorEx inBetweenMotor, shootMotor, shootMotorOp,intakeMotor; //odometry is for testing purposes
    public static Telemetry dashboardTelemetry;
    public static Limelight3A ll;
    public static FtcDashboard dashboard;

    public static GoBildaPinpointDriver odometry;
    public InitMotors(HardwareMap hardwareMap){
        this.hardwareMap = hardwareMap;
    }
    public void initDriveTrain(){ // the initialisation of the drivetrain motor is in pedro
        FL = hardwareMap.get(DcMotorEx.class, "FL");
        FR = hardwareMap.get(DcMotorEx.class, "FR");
        BL = hardwareMap.get(DcMotorEx.class, "BL");
        BR = hardwareMap.get(DcMotorEx.class, "BR");
    }
    
    public void initIntake(){
        intakeMotor = hardwareMap.get(DcMotorEx.class,"Intake");
        intakeMotor.setDirection(DcMotorSimple.Direction.REVERSE);
    }
    
    public void initInBetween(){
        SL = hardwareMap.get(CRServo.class,"SL");
        SL.setDirection(CRServo.Direction.REVERSE);
        SR = hardwareMap.get(CRServo.class,"SR");
        SR.setDirection(CRServo.Direction.REVERSE);

        inBetweenMotor = hardwareMap.get(DcMotorEx.class, "inbetween");
        inBetweenMotor.setDirection(DcMotorSimple.Direction.REVERSE);
    }
    
    public void initShooter(){
        shootMotorOp = hardwareMap.get(DcMotorEx.class, "ShooterFar");
        shootMotorOp.setDirection(DcMotorSimple.Direction.FORWARD);
        shootMotorOp.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);

        shootMotor = hardwareMap.get(DcMotorEx.class, "ShooterClose");
        shootMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        shootMotor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
    }
    
    public void initPinpoint(){ // the initialisation of the pinpoint is in pedro
        odometry = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");
    }
    
    public void initDashboard(){
        dashboard = FtcDashboard.getInstance();
        dashboardTelemetry = dashboard.getTelemetry();
    }

    public void initLL() {
        ll = hardwareMap.get(Limelight3A.class, "limelight");
        ll.setPollRateHz(100); // This sets how often we ask Limelight for data (100 times per second)
        ll.pipelineSwitch(2);
    }
}
