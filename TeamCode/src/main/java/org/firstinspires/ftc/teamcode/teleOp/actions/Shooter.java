package org.firstinspires.ftc.teamcode.teleOp.actions;

import com.acmerobotics.dashboard.config.Config;
import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@Configurable
@Config
public class Shooter {
    public DcMotorEx shooter, shooter2;
    Telemetry telemetry;
    public Shooter(DcMotorEx shootMotor, Telemetry telemetry, DcMotorEx shooter2){
        this.shooter = shootMotor;
        this.telemetry = telemetry;
        this.shooter2 = shooter2;
    }

    // g - gravity acceleration
    final double g = 9.81;
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
        t = Math.sqrt((2/g)*(Math.tan(theta)*d - (h-robot_Height)));
        shoot(theta,d,t);
    }

    double hDiff = h - robot_Height;
    public void shootByAngleOnline(double d){
        theta = 0.804; // in radians
        double cosSquared = Math.cos(theta)*Math.cos(theta);
        double velocity = Math.sqrt((g*d*d)/(2*(cosSquared)*(d*Math.tan(theta)-hDiff)));
        motorPower = 60*velocity/(diameter*Math.PI*MAX_RPM);
        shooter.setPower(motorPower);
        shooter2.setPower(-motorPower);
        telemetry.addData("power", motorPower);
        telemetry.addData("time",t);
        telemetry.addData("theta", theta);
        telemetry.addData("velocity", velocity);
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

    // GPT CODE ----------------------------------------------------------------------------

    public void shootByAngleGPT(double d) {
        theta = 0.804; // radians
        double v = getLaunchVelocity(d, theta);
        setShooterPower(v);
    }

    public double getLaunchVelocity(double d, double theta) {
        // g = 9.81 m/s^2
        double g = 9.81;
        // h = height difference between shooter and target, meters
        return (d / Math.cos(theta)) * Math.sqrt(g / (2 * (d * Math.tan(theta) + (h-robot_Height))));
    }

    public void setShooterPower(double velocity) {
        // Convert linear velocity (m/s) to motor power
        // diameter in meters, MAX_RPM motor constant
        motorPower = (60 * velocity) / (diameter * Math.PI * MAX_RPM);

        shooter.setPower(motorPower);
        shooter2.setPower(-motorPower);

        telemetry.addData("motor power", motorPower);
        telemetry.addData("velocity (m/s)", velocity);
    }
}