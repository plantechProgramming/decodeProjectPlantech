package org.firstinspires.ftc.teamcode.teleOp.actions;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.teleOp.PID;

import com.qualcomm.robotcore.hardware.PIDCoefficients;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.util.ElapsedTime;

import dev.nextftc.core.commands.Command;

@Configurable
@Config
public class Shooter {
    public DcMotorEx shooter, shooter2;
    Telemetry telemetry;
    public static double kP = 215; //og = 215
    public static double kI = 0.5;//0.5
    public static double kD = 0; //0
    public static double kF = 14.5; // OG = 14.5
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

    public double Szonedis;
    public final double errorFix = 1.18; // og = 1.18
    public void noPhysShoot(double x){
        PIDFCoefficients pidNew = new PIDFCoefficients(kP, kI, kD,kF);

        shooter.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidNew);
        shooter2.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidNew);

        shooter.setPower(x);
        shooter2.setPower(-x);

        telemetry.addData("velocity - firsts", shooter.getVelocity());
        telemetry.addData("velocity shooter 1", shooterVelocity.getVelocityFilter());
        telemetry.addData("velocity shooter 2", Math.abs(shooter2Velocity.getVelocityFilter()));
        telemetry.addData("velocity noisy", getVelocity(shooter));
        telemetry.addData("wanted", x*6000);
    }

    int count = 0;
    boolean prevMore = false;
    boolean prevLess = false;

    double power = 0;
    public void variableSpeedShoot(boolean more, boolean less, double jumps){

        if(more && !prevMore){power += jumps;}
        else if(less && !prevLess){
            power -= jumps;
        }
        else{
            telemetry.addData("wanted variable", power*6000);
            prevLess = less;
            prevMore = more;
            return;
        }
        if (power >= 0.7){
            power = 0.7;
        }
        prevLess = less;
        prevMore = more;
        PIDFCoefficients pidNew = new PIDFCoefficients(kP, kI, kD,kF);

        shooter.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidNew);
        shooter2.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidNew);

        shooter.setPower(power*errorFix);
        shooter2.setPower(-power*errorFix);
        telemetry.addData("wanted variable", power*6000);
        telemetry.addData("wanted fixed", errorFix*power*6000);
    }
    public void setDashBoardPID(){
        PIDFCoefficients pidNew = new PIDFCoefficients(kP, kI, kD,kF);
        shooter.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidNew);
        shooter2.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidNew);
    }
//    / @param dis: distance from goal
    /// shoots with different powers depending on what launch zone youre in
    // TODO: make depend on odo vals, closed loop control for values
    public void naiveShooter(boolean far) {
        PIDFCoefficients pidNew = new PIDFCoefficients(kP, kI, kD,kF);

        shooter.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidNew);
        shooter2.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidNew);
        if (!far) {
            Szonedis = 0.475;
        } else{
            Szonedis = 0.552;
        }
        shooter.setPower(Szonedis*errorFix);
        shooter2.setPower(-Szonedis*errorFix);

        telemetry.addData("velocity shooter ", shooterVelocity.getVelocityFilter());
        telemetry.addData("wanted", Szonedis*6000);
    }
    public void stopShooter(){
       shooter.setPower(0);
       shooter2.setPower(0);
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

    public void setShooterTelemetry(Telemetry telemetry){
        telemetry.addData("wanted", Szonedis*6000);
        telemetry.addData("vel",shooterVelocity.getVelocityFilter());
        telemetry.addData("vel2", Math.abs(shooter2Velocity.getVelocityFilter()));
//        telemetry.addData("th",Math.abs(shooterVelocity.getVelocityFilter() - Szonedis*6000));
        telemetry.addData("pow 1", shooter.getPower());
        telemetry.addData("pow 2", shooter2.getPower());
        PIDFCoefficients coefficients = shooter.getPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER);
//        telemetry.addData("p", coefficients.p);
//        telemetry.addData("i", coefficients.i);
//        telemetry.addData("d", coefficients.d);
    }

    public boolean isUpToSpeed(){
        double threshold = 130; //TODO: tune!! should be the biggest reliably scoring value
        return Math.abs(shooterVelocity.getVelocityFilter() - Szonedis*6000) < threshold;
    }
    public void out(){
        shooter.setPower(-.2);
        shooter2.setPower(.2);
    }
}