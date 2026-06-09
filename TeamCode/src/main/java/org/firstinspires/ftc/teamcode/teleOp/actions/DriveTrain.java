package org.firstinspires.ftc.teamcode.teleOp.actions;

import android.util.Pair;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import com.seattlesolvers.solverslib.controller.Controller;
import com.seattlesolvers.solverslib.controller.PIDFController;


import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.auto.camera.AprilTagLocalization;
import org.firstinspires.ftc.teamcode.teleOp.PID;
import org.firstinspires.ftc.teamcode.teleOp.Utils;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.opencv.core.Mat;
import org.openftc.easyopencv.OpenCvCamera;

@Config
public class DriveTrain {
    private DcMotorEx BR, BL, FR, FL;
    private BNO055IMU imu;
    private IMU Imu;
    private Telemetry telemetry;
    private LinearOpMode opMode;
    private OpenCvCamera camera;
    private GoBildaPinpointDriver odometry;
    ElapsedTime runtime = new ElapsedTime();
    Utils utils;
    public static double Kp = 0.023, Ki = 0, Kd = 1.8, Kf = 0, Ks = 0.04; // prev kp = 0.0325, ki = 0, kd = 2.1, kf=0
    static final double WHEEL_DIAMETER_CM = 10.4;     // For figuring circumference
    private PID pid;
    AprilTagLocalization tagLocalization;
    String team;
    VoltageSensor voltageSensor;
//    private PID shortTurnPID;
    static final double COUNTS_PER_CM = 537.6 / WHEEL_DIAMETER_CM * Math.PI;//(COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_CM * PI);

    public DriveTrain(DcMotorEx BR, DcMotorEx BL, DcMotorEx FR, DcMotorEx FL, Telemetry telemetry, IMU imu, GoBildaPinpointDriver odometry, String team, Utils utils) {
        this.BL = BL;
        this.BR = BR;
        this.FL = FL;
        this.FR = FR;
        this.odometry = odometry;
        this.Imu = imu;
        this.telemetry = telemetry;
        this.team = team;
        this.utils = utils;
        pid = new PID(Kp, Ki, Kd, Kf, Ks);// prev GOOD p = 0.022, i = 0.00000001, d = 0.000001, f = 0
        tagLocalization = new AprilTagLocalization(team,telemetry);
    }


    public void side_drive(double power){
        FL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        FR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        BR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        BL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


        FL.setPower(1 * power);
        BL.setPower(-1 * power);

        FR.setPower(-1 * power);
        BR.setPower(1 * power);
    }

    public void drive(double y, double x, double rx, double botHeading, double slowRatio){
        double rxUsable = rx;

        // slowRatio [0,1] - output power multiplier


        // Rotate the movement direction counter to the bot's rotation
        double rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
        double rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);

        rotX *= 1.1;  // Counteract imperfect strafing

        if(turnPow != 0){
            rxUsable = -turnPow;
        }
        // Denominator is the largest motor power (absolute value) or 1
        // This ensures all the powers maintain the same ratio,
        // but only if at least one is out of the range [-1, 1]
        double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rxUsable), 1);
        double frontLeftPower = (rotY + rotX + rxUsable) / denominator;
        double backLeftPower = (rotY - rotX + rxUsable) / denominator;

        double frontRightPower = (rotY - rotX - rxUsable) / denominator;// before - rotX
        double backRightPower = (rotY + rotX - rxUsable) / denominator;// before + rotX

        FL.setPower(frontLeftPower * slowRatio);
        BL.setPower(backLeftPower * slowRatio);

        FR.setPower(frontRightPower * slowRatio);
        BR.setPower(backRightPower * slowRatio);
        turnPow = 0;

    }
    
    public void stop(){
        FL.setPower(0);
        FR.setPower(0);
        BR.setPower(0);
        BL.setPower(0);
    }
    double count;
    double error;

    double turnPow = 0;
    public void turnToGyro(double degrees, double voltage) {
        double botAngleRaw = odometry.getHeading(AngleUnit.DEGREES);

        double threshold = 0.5;
        double power = 0;
        pid.setWanted(degrees);
        if(Math.abs(utils.getDiffBetweenAngles(degrees, botAngleRaw)) > threshold) { // if not in threshold
            power = pid.updatedeg(botAngleRaw);
            error = degrees - botAngleRaw;

//        }
//        else{
//            power = 0;
//        }
            power = utils.getVoltageCompensatedPow(power, voltage);
        }
//        FL.setPower(-power);
//        FR.setPower(power);
//
//        BR.setPower(power);
//        BL.setPower(-power);
//        telemetry.addData("pow", power);
//        telemetry.addData("wanted", degrees);
//        telemetry.addData("curr", botAngleRaw);
        turnPow = power;
//        telemetry.addData("heading", botAngleRaw);
    }
    public void turnTowardsAprilTag(AprilTagDetection tag){
        double error = tag.ftcPose.bearing;
        double pow = pid.getPIDPower(error); // not a loop around pid, cuz how could we detect apriltags that far?
        FL.setPower(-pow);
        FR.setPower(pow);

        BR.setPower(pow);
        BL.setPower(-pow);
        telemetry.addData("pow using April tag", pow);
    }
    double deg = 0;
    public boolean usingCamForTurn = false;
    public void turnToGoal(String team, AprilTagDetection goalTag){
        if(goalTag != null){
            turnTowardsAprilTag(goalTag);
            usingCamForTurn = true;
        }
        else if(!usingCamForTurn){
            deg = utils.getAngleFromGoal(team);
            turnToGyro(deg, voltageSensor.getVoltage());
        }
        else{ // start of endless turning
            usingCamForTurn = false;
        }
    }

    public boolean isStopped(){
        boolean xInThresh = Math.abs(odometry.getVelX(DistanceUnit.CM)) < 5;
        boolean yInThresh = Math.abs(odometry.getVelY(DistanceUnit.CM)) < 5;
        boolean headInThresh = Math.abs(odometry.getHeadingVelocity(AngleUnit.DEGREES.getUnnormalized())) < 3;
        return xInThresh && yInThresh && headInThresh;
    }


    public void setDriveTelemetry(Telemetry telemetry){
        telemetry.addData("botheading", odometry.getHeading(AngleUnit.DEGREES));
//        telemetry.addData("deg to goal red",utils.getAngleFromGoal("RED"));
        telemetry.addData("deg to goal blue",utils.getAngleFromGoal("BLUE"));
//        telemetry.addData("dis xy to goal red",utils.getXYdistToGoal("RED"));
//        telemetry.addData("dis xy to goal blue",utils.getXYdistToGoal("BLUE"));
//        telemetry.addData("disToGoalred", utils.getDistFromGoal("RED"));
//        telemetry.addData("disToGoalblue", utils.getDistFromGoal("BLUE"));
//        telemetry.addData("robot is stopping", isStopped());
//        telemetry.addData("mode", FL.getZeroPowerBehavior());
//        telemetry.addData("botheadingIMU",Imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES));
        telemetry.addData("X pos: ", odometry.getPosX(DistanceUnit.CM));
        telemetry.addData("Y pos: ", odometry.getPosY(DistanceUnit.CM));
//        telemetry.addData("is far", utils.isFar());
//        telemetry.addData("blue goal", utils.GOAL_BLUE);
//        telemetry.addData("blue goal far", utils.GOAL_BLUE_FAR);
//        telemetry.addData("blue goal close", utils.GOAL_BLUE_CLOSE);
//        telemetry.addData("voltage", voltageSensor.getVoltage());
//        telemetry.addData("error deg", error);
//        telemetry.addData("wrap around error", utils.convertToWrapAroundAngle(error));
    }
}

