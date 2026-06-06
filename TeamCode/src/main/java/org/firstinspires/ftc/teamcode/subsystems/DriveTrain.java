package org.firstinspires.ftc.teamcode.subsystems;

import static com.pedropathing.ivy.groups.Groups.parallel;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.geometry.Pose;
import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.commands.Commands;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;


import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.Misc.Utils.AngleFunctions;
import org.firstinspires.ftc.teamcode.Misc.Utils.Extras;
import org.firstinspires.ftc.teamcode.Misc.Utils.TelemetryUtils;
import org.firstinspires.ftc.teamcode.subsystems.Camera.AprilTagLocalization;
import org.firstinspires.ftc.teamcode.Misc.PID;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;

@Config
public class DriveTrain {
    private DcMotorEx BR, BL, FR, FL;
    private BNO055IMU imu;
    private IMU Imu;
    private Telemetry telemetry;
    private GoBildaPinpointDriver odometry;
    ElapsedTime runtime = new ElapsedTime();
    public static double Kp = 0.034, Ki = 3e-9, Kd = 2.5, Kf = 0; // prev kp = 0.0325, ki = 0, kd = 2.1, kf=0
    public static int t = 1;
    static final double WHEEL_DIAMETER_CM = 10.4;     // For figuring circumference
    private PID pid;
    String team;
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
        pid = new PID(Kp, Ki, Kd, Kf,t, this.telemetry);// prev GOOD p = 0.022, i = 0.00000001, d = 0.000001, f = 0
    }

    public Command drive(double y, double x, double rx, double botHeading, double slowRatio){

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

        telemetry.addData("FL", frontLeftPower);
        telemetry.addData("FR", frontRightPower);
        telemetry.addData("BR", backRightPower);
        telemetry.addData("BL", backLeftPower);

        return setPower(
                frontLeftPower * slowRatio,
                backLeftPower * slowRatio,
                frontRightPower * slowRatio,
                backRightPower * slowRatio
        );

    }
    
    public void stop(){
        FL.setPower(0);
        FR.setPower(0);
        BR.setPower(0);
        BL.setPower(0);
    }
    double turnPow = 0;
    public Command turnToAngle(double wantedDeg, double curDeg) {

        double threshold = 0.55;
        turnPow = 0;
        pid.setWanted(wantedDeg);
//        if(Math.abs(utils.getDiffBetweenAngles(degrees, botAngleRaw)) > threshold){ // if not in threshold
        turnPow = pid.updatedeg(curDeg);
//        }
//        else{
//            power = 0;
//        }
        return turnWithPow(turnPow);
    }

    public Command setPower(double FLPow, double BLPow, double FRPow, double BRPow){
        return parallel(
                Commands.instant(()->FL.setPower(FLPow)),
                Commands.instant(()->BL.setPower(BLPow)),
                Commands.instant(()->FR.setPower(FRPow)),
                Commands.instant(()->BR.setPower(BRPow))
        );
    }
    public Command turnWithPow(double pow){
        return setPower(-pow, -pow, pow, pow);
    }

    public void updateTelemetry(Telemetry telemetry){
        TelemetryUtils.addTitle(telemetry, "starting drive telemetry");
        TelemetryUtils.addVar(telemetry, turnPow);
        TelemetryUtils.addTitle(telemetry, "ending drive telemetry");
    }
}

