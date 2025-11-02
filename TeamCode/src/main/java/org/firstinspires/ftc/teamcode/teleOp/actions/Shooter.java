package org.firstinspires.ftc.teamcode.teleOp.actions;

import com.acmerobotics.dashboard.config.Config;
import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import dev.nextftc.core.units.Distance;

@Configurable
@Config
public class Shooter {
    public DcMotorEx shooter, shooter2;
    Telemetry telemetry;
    public <roni2_intake> Shooter(DcMotorEx shootMotor, Telemetry telemetry, DcMotorEx shooter2){
        this.shooter = shootMotor;
        this.telemetry = telemetry;
        this.shooter2 = shooter2;
    }

    // g - gravity acceleration
    final double g = 9.8;
    // h - goal height + some 5 cm. IN CM
    final double h = 1.15;

    final double robot_Height = 0.4;
    final double diameter = .096; //in m
    final int MAX_RPM = 6000;
    public void noPhysShoot(double x){
        shooter.setPower(x);
        shooter2.setPower(-x);
    }

    public double motorPower;
    public double theta;
    public double t;

    public void shootByTime(double d, double t){
        theta = Math.atan((g*t*t + 2*h)/(2*d));
        shoot(theta,d,t);
    }

    public void shootByAngle(double d){
        // TODO: make cases for different odo vals, test if even needed
        theta = 0.804; // in radians
        t = Math.sqrt((2/g)*(Math.tan(theta)*d - h));
        shoot(theta,d,t);
    }

    public void shoot(double theta, double d, double t){
        double velocity = 2*d/(Math.cos(theta)*t);
        motorPower = 60*velocity/(diameter*Math.PI*MAX_RPM);
        shooter.setPower(motorPower);
        shooter2.setPower(-motorPower);
        telemetry.addData("power", motorPower);
        telemetry.addData("time",t);
        telemetry.addData("theta", theta);
        telemetry.addData("velocity", velocity);
    }

}