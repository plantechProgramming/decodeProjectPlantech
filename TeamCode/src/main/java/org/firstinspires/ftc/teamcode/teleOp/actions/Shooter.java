package org.firstinspires.ftc.teamcode.teleOp.actions;

import com.acmerobotics.dashboard.config.Config;
import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.teleOp.PID;

import com.qualcomm.robotcore.hardware.PIDCoefficients;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.util.ElapsedTime;

@Configurable
@Config
public class Shooter {
    public DcMotorEx shooter, shooter2;
    Telemetry telemetry;
    public static double kP = 60;
    public static double kI = 1;
    public static double kD = 8;
    public static double kF = 0;
    GetVelocity shooterVelocity;
    GetVelocity shooter2Velocity;

    public Shooter(DcMotorEx shootMotor, Telemetry telemetry, DcMotorEx shooter2){
        this.shooter = shootMotor;
        this.telemetry = telemetry;
        this.shooter2 = shooter2;

        shooterVelocity = new GetVelocity(shooter,0.1);
        shooter2Velocity = new GetVelocity(shooter2,0.1);
    }

    // g - gravity acceleration
    final double g = 9.81;
    // h - goal height + some 5 cm. IN CM
    final double h = 1.15;
    final double robot_Height = 0.4;
    final double diameter = .096; //in m
    final int MAX_RPM = 6000;

    double Szonedis;
    final double errorFix = 1.18;

    public void noPhysShoot(double x){
        shooter.setPower(x);
        shooter2.setPower(-x);
        telemetry.addData("velocity - firsts", shooter.getVelocity());
        telemetry.addData("velocity shooter 1", shooterVelocity.getVelocityFilter());
        telemetry.addData("velocity shooter 2", shooter2Velocity.getVelocityFilter());
        telemetry.addData("velocity noisy", getVelocity(shooter));
        telemetry.addData("wanted", x*6000);
    }

    double prevPower = 0;
    int count = 0;
    public void variableSpeedShoot(boolean dpadUp, boolean dpadDown, double jumps){
        // make func run only once per 10 calls, so that when pressed once it wont go up so much
        //TODO: remove, delete, destroy, annihilate, die code die, install GNU/Linux, sudo rm -rf --no-preserve-root /, go to code hell, kill; just joking
//        if(!(count % 10 == 0)){
//            count++;
//            return;
//        }
        double power = 0;
        if(dpadUp){power = prevPower + jumps;}
        else if(dpadDown){power = prevPower - jumps;}
        else{power = prevPower;}

        shooter.setPower(power);
        shooter2.setPower(-power);
        telemetry.addData("power", power*6000);
        telemetry.addData("velocity", shooterVelocity.getVelocityFilter());
        prevPower = power;
    }

    /// @param dis: distance from goal
    /// shoots with different powers depending on what launch zone youre in
    // TODO: make depend on odo vals, closed loop control for values
    public void naiveShooter(double dis) {
        PIDFCoefficients pidNew = new PIDFCoefficients(kP, kI, kD,kF);

        shooter.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidNew);
        shooter2.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidNew);
        if (dis <= 1.3) {
            Szonedis = 0.44;
        } else{
            Szonedis = 0.55;
        }
        shooter.setPower(Szonedis*errorFix);
        shooter2.setPower(-Szonedis*errorFix);

        telemetry.addData("velocity shooter ", shooterVelocity.getVelocityFilter());
        telemetry.addData("wanted", Szonedis*6000);
    }


     // velocity ---------------------------------------------------------------------------


    private static final ElapsedTime timer = new ElapsedTime();
    public long prevEncoder = 0;
    public long curEncoder;

    public double prevTime = 0;
    public double curTime;
    int ticksPerRevolution = 28;
    int millisecondsToMinute = 60000;
    public double getVelocity(DcMotorEx motor){
        curEncoder = motor.getCurrentPosition();
        curTime = timer.milliseconds();

        double timeDiff = curTime - prevTime;
        double encoderDiff = curEncoder - prevEncoder;

        double tickVelocity = encoderDiff/timeDiff; // ticks/milliseconds
        double velocity = (tickVelocity*millisecondsToMinute)/ticksPerRevolution;

        prevTime = curTime;
        prevEncoder = curEncoder;

        telemetry.addData("time",curTime);
        return velocity;
    }
    double prevVelocity = 0;
    double alpha = 0.1;

    public double getVelocityFilter(DcMotorEx motor) {
        curEncoder = motor.getCurrentPosition();
        curTime = timer.milliseconds();

        double timeDiff = curTime - prevTime;
        double encoderDiff = curEncoder - prevEncoder;

        double tickVelocity = encoderDiff / timeDiff; // ticks/milliseconds
        double velocity = (tickVelocity * millisecondsToMinute) / ticksPerRevolution;

        prevTime = curTime;
        prevEncoder = curEncoder;
        double filteredVelocity = alpha*velocity + (1-alpha)*prevVelocity;
        prevVelocity = filteredVelocity;
        return filteredVelocity;
    }
    double avgPrevVel = 0;
    public double getVelocityAverage(DcMotorEx motor){
        curEncoder = motor.getCurrentPosition();
        curTime = timer.milliseconds();

        double timeDiff = curTime - prevTime;
        double encoderDiff = curEncoder - prevEncoder;

        double tickVelocity = encoderDiff/timeDiff; // ticks/milliseconds
        double velocity = (tickVelocity*millisecondsToMinute)/ticksPerRevolution;

        prevTime = curTime;
        prevEncoder = curEncoder;
        double velAvg = (avgPrevVel*count+velocity)/(count + 1);
        avgPrevVel = velAvg;
        count++;
        return velAvg;
    }

    // OLD -------------------------------------------------------------------------------------

    public double motorPower;
    public double theta;
    public double t;

    double hDiff = h - robot_Height;
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
        double velocity = 2*d/(Math.acos(theta)*t);
        motorPower = 60*velocity/(diameter*Math.PI*MAX_RPM);
        shooter.setPower(motorPower);
        shooter2.setPower(-motorPower);
        telemetry.addData("power", motorPower);
        telemetry.addData("time",t);
        telemetry.addData("theta", theta);
        telemetry.addData("velocity", velocity);
    }

}