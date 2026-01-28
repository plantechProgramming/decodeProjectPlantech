package org.firstinspires.ftc.teamcode.teleOp.actions;

import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.teleOp.PID;
import org.opencv.core.Mat;
import org.openftc.easyopencv.OpenCvCamera;

public class DriveTrain {
    private DcMotorEx BR, BL, FR, FL;
    private BNO055IMU imu;
    private IMU Imu;
    private Telemetry telemetry;
    private LinearOpMode opMode;
    private OpenCvCamera camera;
    private GoBildaPinpointDriver odometry;
    ElapsedTime runtime = new ElapsedTime();
    public static double Kp = 0.5, Ki = 0.2, Kd = 0.01;
    static final double WHEEL_DIAMETER_CM = 10.4;     // For figuring circumference
    static final double COUNTS_PER_CM = 537.6 / WHEEL_DIAMETER_CM * Math.PI;//(COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_CM * PI);

    public DriveTrain(DcMotorEx BR, DcMotorEx BL, DcMotorEx FR, DcMotorEx FL, Telemetry telemetry, IMU imu, GoBildaPinpointDriver odometry) {
        this.BL = BL;
        this.BR = BR;
        this.FL = FL;
        this.FR = FR;
        this.odometry = odometry;
        this.Imu = imu;

        this.telemetry = telemetry;
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
    public void turnToGyro(double degrees) {
        double botAngleRaw = odometry.getHeading(AngleUnit.DEGREES);
        PID pid = new PID(0.022, 0.00000001, 0.000001, 0); // prev GOOD p = 0.022, i = 0.00000001, d = 0.000001, f = 0
        double threshold = 0.5;
        double power = 0;
        pid.setWanted(degrees);

        if(Math.abs(degrees - botAngleRaw) > threshold){ // if not in threshold

            power = pid.updatedeg(botAngleRaw);
            FL.setPower(-power);
            FR.setPower(power);

            BR.setPower(power);
            BL.setPower(-power);
        }
        telemetry.addData("pow", power);
//        telemetry.addData("heading", botAngleRaw);
    }
    double deg = 0;
    public void turnToGoal(String team){
        double lenfield = 365; // cm
        double x = odometry.getPosX(DistanceUnit.CM);
        double y = odometry.getPosY(DistanceUnit.CM);
        double yOffset = 10;//prev = 18
        double xOffset = 0; // prev = 16
        if(team == "RED"){
            x = lenfield/2 + x;
        }
        else{
            x = lenfield/2 - x;
        }

        deg = Math.toDegrees(Math.atan2((lenfield/2 + y - yOffset), (x - xOffset)));
        if(team == "BLUE"){
            deg = -deg;
        }
        else{
            deg = 180 + deg;
        }
        turnToGyro(deg);
    }

    public boolean isFar(){
        return odometry.getPosY(DistanceUnit.CM) > 60;
    }

    public void setDriveTelemetry(Telemetry telemetry){
        telemetry.addData("botheading",odometry.getHeading(AngleUnit.DEGREES));
        telemetry.addData("deg to goal",deg);
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

