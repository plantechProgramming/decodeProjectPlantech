package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.CameraName;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;


public abstract class TeamOpMode extends LinearOpMode {
    InitMotors initMotors;

    protected CRServo SL,SR;
    protected DcMotorEx FL, FR, BL, BR,inBetweenMotor, shootMotor, shootMotorOp,intakeMotor; //odometry is for testing purposes
    protected Telemetry dashboardTelemetry;
    protected GoBildaPinpointDriver odometry;

    private void initAll(){
        initMotors.initDriveTrain();
        initMotors.initIntake();
        initMotors.initInBetween();
        initMotors.initShooter();
        initMotors.initPinpiont();
        initMotors.initDashboard();
    }

    private void initMotors(){
        FL = InitMotors.FL; FR = InitMotors.FR; BL = InitMotors.BL; BR = InitMotors.BR;
        SL = InitMotors.SL; SR = InitMotors.SR;
        inBetweenMotor = InitMotors.inBetweenMotor;
        shootMotor = InitMotors.shootMotor; shootMotorOp = InitMotors.shootMotorOp;
        intakeMotor = InitMotors.intakeMotor;
        dashboardTelemetry = InitMotors.dashboardTelemetry;
        odometry = InitMotors.odometry;

    }
    @Override
    public void runOpMode() throws InterruptedException  {
        initMotors = new InitMotors(hardwareMap);
        this.initMotors();
        initAll();
        waitForStart();
        postInit();

        if (opModeIsActive()) {
            run();
        }

        end();
    }

    protected void postInit() {

    }
    protected abstract void run();

    protected abstract void end();
}

