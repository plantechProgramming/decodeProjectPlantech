package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Misc.InitMotors;


public abstract class TeamOpMode extends LinearOpMode {
    InitMotors initMotors;

    protected CRServo SL,SR;
    protected DcMotorEx FL, FR, BL, BR,inBetweenMotor, shootMotor, shootMotorOp,intakeMotor;
    protected Telemetry dashboardTelemetry;
    protected GoBildaPinpointDriver odometry;
    protected Limelight3A ll;

    private void initAll(){
        initMotors.initDriveTrain();
        initMotors.initIntake();
        initMotors.initInBetween();
        initMotors.initShooter();
        initMotors.initPinpoint();
        initMotors.initDashboard();
        initMotors.initLL();
    }

    private void initMotors(){
        FL = InitMotors.FL; FR = InitMotors.FR; BL = InitMotors.BL; BR = InitMotors.BR;
        SL = InitMotors.SL; SR = InitMotors.SR;
        inBetweenMotor = InitMotors.inBetweenMotor;
        shootMotor = InitMotors.shootMotor; shootMotorOp = InitMotors.shootMotorOp;
        intakeMotor = InitMotors.intakeMotor;
        dashboardTelemetry = InitMotors.dashboardTelemetry;
        odometry = InitMotors.odometry;
        ll = InitMotors.ll;
        telemetry = new MultipleTelemetry(telemetry, dashboardTelemetry);
        InitMotors.dashboard.startCameraStream(ll, 60);
    }
    @Override
    public void runOpMode() throws InterruptedException  {
        initMotors = new InitMotors(hardwareMap);
        initAll();
        initMotors();
        postInit();
        waitForStart();

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

