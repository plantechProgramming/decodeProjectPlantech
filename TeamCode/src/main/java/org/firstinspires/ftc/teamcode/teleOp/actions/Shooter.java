package org.firstinspires.ftc.teamcode.teleOp.actions;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import dev.nextftc.core.units.Distance;

@Configurable
public class Shooter {
    DcMotorEx shooter, shooter2;
    Telemetry telemetry;
    public <roni2_intake> Shooter(DcMotorEx shootMotor, Telemetry telemetry, DcMotorEx shooter2){
        this.shooter = shootMotor;
        this.telemetry = telemetry;
        this.shooter2 = shooter2;
    }

    // g - gravity acceleration
    final double g = 9.8;
    // h - goal height + some 5 cm. IN CM
    final double h = 0.95;
    final double diameter = .096; //in m
    final int MAX_RPM = 6000;
    public void noPhysShoot(double x){
        shooter2.setPower(-x);
        shooter.setPower(x);
    }

    public double motorPower;
    public double theta;
    public double t;

    public void shootByTime(double d, double t){
        theta = Math.atan((g*t*t + 2*h)/(2*d));
        // the artifact turns between a stationary wall and the flywheel, so you
        // need to multiply by 2
        double velocity = 2*d/(Math.cos(theta)*t);
        motorPower = 60*velocity/(diameter*Math.PI*MAX_RPM);
        telemetry.addData("velocity",velocity);
        telemetry.addData("motorPower", motorPower);
        shooter.setPower(motorPower);
        shooter2.setPower(-motorPower);
    }

    public void shootByAngle(double d){
        // TODO: make cases for different odo vals, test if even needed
        theta = 1.16; // in radians
        t = Math.sqrt((2/g)*(Math.tan(theta)*d - h));
        double velocity = 2*d/(Math.cos(theta)*t);
        telemetry.addData("velocity",velocity);
        telemetry.addData("time", t);
        motorPower = 60*velocity/(diameter*Math.PI*MAX_RPM);
        telemetry.addData("motorPower", motorPower*(2));
        shooter.setPower(motorPower*(2));
        shooter2.setPower(-motorPower*(2));
    }
}
