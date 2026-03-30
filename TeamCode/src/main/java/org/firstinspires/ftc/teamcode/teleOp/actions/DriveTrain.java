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
    public static double Kp = 0.034, Ki = 3e-9, Kd = 2.5, Kf = 0; // prev kp = 0.0325, ki = 0, kd = 2.1, kf=0
    public static int t = 1;
    static final double WHEEL_DIAMETER_CM = 10.4;     // For figuring circumference
    private PID pid;
    AprilTagLocalization tagLocalization;
    String team;
//    private PID shortTurnPID;
    static final double COUNTS_PER_CM = 537.6 / WHEEL_DIAMETER_CM * Math.PI;//(COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_CM * PI);

    public DriveTrain(DcMotorEx BR, DcMotorEx BL, DcMotorEx FR, DcMotorEx FL, Telemetry telemetry, IMU imu, GoBildaPinpointDriver odometry, String team) {
        this.BL = BL;
        this.BR = BR;
        this.FL = FL;
        this.FR = FR;
        this.odometry = odometry;
        this.Imu = imu;
        this.telemetry = telemetry;
        this.team = team;
        this.utils = new Utils(this.telemetry, this.odometry);
        pid = new PID(Kp, Ki, Kd, Kf,t, this.telemetry);// prev GOOD p = 0.022, i = 0.00000001, d = 0.000001, f = 0
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

        // slowRatio [0,1] - output power multiplier

        FL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        FR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        BR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        BL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Rotate the movement direction counter to the bot's rotation
        double rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
        double rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);

        rotX *= 1.1;  // Counteract imperfect strafing

        // Denominator is the largest motor power (absolute value) or 1
        // This ensures all the powers maintain the same ratio,
        // but only if at least one is out of the range [-1, 1]
        double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);
        double frontLeftPower = (rotY + rotX + rx) / denominator;
        double backLeftPower = (rotY - rotX + rx) / denominator;

        double frontRightPower = (rotY - rotX - rx) / denominator;// before - rotX
        double backRightPower = (rotY + rotX - rx) / denominator;// before + rotX

        FL.setPower(frontLeftPower * slowRatio);
        BL.setPower(backLeftPower * slowRatio);

        FR.setPower(frontRightPower * slowRatio);
        BR.setPower(backRightPower * slowRatio);

    }
    
    public void stop(){
        FL.setPower(0);
        FR.setPower(0);
        BR.setPower(0);
        BL.setPower(0);
    }
    double count;

    public void turnToGyro(double degrees) {
        double botAngleRaw = odometry.getHeading(AngleUnit.DEGREES);

        double threshold = 0.55;
        double power = 0;
        pid.setWanted(degrees);
//        if(Math.abs(utils.getDiffBetweenAngles(degrees, botAngleRaw)) > threshold){ // if not in threshold
        power = pid.getPIDPowerWithT(botAngleRaw);
//        }
//        else{
//            power = 0;
//        }
        FL.setPower(-power);
        FR.setPower(power);

        BR.setPower(power);
        BL.setPower(-power);
        telemetry.addData("pow", power);
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
            turnToGyro(deg);
        }
        else{ // start of endless turning
            usingCamForTurn = false;
        }
    }
    public void turnToGoalWithThresh(String team, AprilTagDetection goalTag){
        double thresh = 1;
        if(goalTag != null){
            if(!(Math.abs(goalTag.ftcPose.bearing) < thresh)){
                turnToGoal(team, goalTag);
            }
            else{
/*
                stop();
*/
            }
        }
        else if(!(Math.abs(utils.getAngleFromGoal(team)-odometry.getHeading(AngleUnit.DEGREES)) < thresh)){
            turnToGoal(team, goalTag);
        }
        else{
//            stop();
            telemetry.addLine("aaaaaaaa");
        }
    }

    public boolean isFar(){
        return odometry.getPosY(DistanceUnit.CM) > 60;
    }
    public boolean isStopped(){
        boolean xInThresh = Math.abs(odometry.getVelX(DistanceUnit.CM)) < 3;
        boolean yInThresh = Math.abs(odometry.getVelY(DistanceUnit.CM)) < 3;
        boolean headInThresh = Math.abs(odometry.getHeadingVelocity(AngleUnit.DEGREES.getUnnormalized())) < 2;
        return xInThresh && yInThresh && headInThresh;
    }

    public Pose2D lastFilteredPose = new Pose2D(DistanceUnit.CM, 0, 0, AngleUnit.DEGREES, 0);
    public Pose2D filteredPose = null;
    public Pose2D filterCamPose(Pose2D pose){
        if(filteredPose != null)
            if(utils.PoseThreshold(filteredPose, odometry.getPosition(), 100, 10)){
                filteredPose = utils.medianPose(pose);
            }else{
                filteredPose = pose;
            }
        else{
            filteredPose = pose;
        }

//        if(utils.PoseThreshold(pose, filteredPose, 15, 10)){ // heading threshold is big because were not using the heading filtered
//            filteredPose = utils.filterPose(0.7, pose, lastFilteredPose);
//        }
        lastFilteredPose = filteredPose;
        return filteredPose;
    }
    public void setDriveTelemetry(Telemetry telemetry){
        telemetry.addData("botheading", odometry.getHeading(AngleUnit.DEGREES));
        telemetry.addData("deg to goal red",utils.getAngleFromGoal("RED"));
        telemetry.addData("disToGoalred", utils.getDistFromGoal("RED"));
        telemetry.addData("disToGoalblue", utils.getDistFromGoal("BLUE"));
        telemetry.addData("robot is stopping", isStopped());
//        telemetry.addData("mode", FL.getZeroPowerBehavior());
//        telemetry.addData("botheadingIMU",Imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES));
        telemetry.addData("X pos: ", odometry.getPosX(DistanceUnit.CM));
        telemetry.addData("Y pos: ", odometry.getPosY(DistanceUnit.CM));
        telemetry.addData("is far", isFar());
    }
    public Pose2D PedroPoseConverter(Pose pose){
        double x = pose.getX();
        double y = pose.getY();
        double hed = Math.toDegrees(pose.getHeading());
        double lenField = 365.76; // 144 inch to cm
        double newx = ((-lenField/144)*x)+lenField/2;
        double newy = ((-lenField/144)*y)+lenField/2;
        hed = hed - 180;
        if(hed <= 180){
            hed += 360;
        }
        return new Pose2D(DistanceUnit.CM, newx, newy, AngleUnit.DEGREES, hed);
    }
}

