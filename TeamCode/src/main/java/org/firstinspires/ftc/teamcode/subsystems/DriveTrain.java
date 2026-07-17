package org.firstinspires.ftc.teamcode.subsystems;

import static com.pedropathing.ivy.groups.Groups.parallel;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.commands.Commands;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;


import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Misc.InitMotors;
import org.firstinspires.ftc.teamcode.Misc.Utils.TelemetryUtils;
import org.firstinspires.ftc.teamcode.Misc.PID;

@Config
public class DriveTrain {
    private DcMotorEx BR, BL, FR, FL;
    public static double Kp = 0.034, Ki = 3e-9, Kd = 2.5, Kf = 0; // prev kp = 0.0325, ki = 0, kd = 2.1, kf=0
    private PID pid;

    public DriveTrain() {
        this.BL = InitMotors.BL;
        this.BR = InitMotors.BR;
        this.FL = InitMotors.FL;
        this.FR = InitMotors.FR;
        pid = new PID(Kp, Ki, Kd, Kf);// prev GOOD p = 0.022, i = 0.00000001, d = 0.000001, f = 0
    }


    public Command drive(double y, double x, double rx, double botHeading, double slowRatio){ // bot heading in degrees
        // slowRatio [0,1] - output power multiplier
        botHeading = Math.toRadians(botHeading);

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
        turnPow = pid.updateDeg(curDeg);
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

    public static void setDriveToBrakeMode(){
        InitMotors.BL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        InitMotors.BR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        InitMotors.FR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        InitMotors.FL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public static void setDriveToFloatMode(){
        InitMotors.BL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        InitMotors.BR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        InitMotors.FR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        InitMotors.FL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
    }

    public void updateTelemetry(Telemetry telemetry){
        telemetry.addData("odo x", InitMotors.odometry.getPosX(DistanceUnit.CM));
        telemetry.addData("odo y", InitMotors.odometry.getPosY(DistanceUnit.CM));
        telemetry.addData("heading", InitMotors.odometry.getHeading(AngleUnit.DEGREES));
        TelemetryUtils.addTitle(telemetry, "starting drive telemetry");
        telemetry.addData("turnPow", turnPow);
        TelemetryUtils.addTitle(telemetry, "ending drive telemetry");
    }
}

