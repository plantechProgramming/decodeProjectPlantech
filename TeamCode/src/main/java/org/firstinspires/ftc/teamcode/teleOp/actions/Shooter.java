package org.firstinspires.ftc.teamcode.teleOp.actions;

//import com.ThermalEquilibrium.homeostasis.Controllers.Feedback.BasicPID;
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.teleOp.PID;

import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.seattlesolvers.solverslib.controller.PIDFController;
//import com.ThermalEquilibrium.homeostasis.Parameters.PIDCoefficients;

import dev.nextftc.control.ControlSystem;
import dev.nextftc.control.KineticState;
import dev.nextftc.core.commands.Command;
import dev.nextftc.hardware.impl.MotorEx;

@Configurable
@Config
public class Shooter {
    PID pid = new PID(0.5, 0, 0, 0);
    public DcMotorEx shooter, shooter2;
    Telemetry telemetry;
    public static double kP = 50; //og = 215
    public static double kI = 0.1;//0.5
    public static double kD = 1; //0
    public static double kF = 0.6; // OG = 14.5


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

        shooter.setPower(x*errorFix);
        shooter2.setPower(-x*errorFix);

        telemetry.addData("velocity - firsts", shooter.getVelocity());
        telemetry.addData("velocity shooter 1", shooterVelocity.getVelocityFilter());
        telemetry.addData("velocity shooter 2", Math.abs(shooter2Velocity.getVelocityFilter()));
        telemetry.addData("velocity noisy", getVelocity(shooter));
        telemetry.addData("wanted", x*6000);
    }
//    PIDFController controller = new PIDFController(0.5,0,0,0);
    public void noPhysShootHomeostasis(double x){
//        double output = controller.calculate(x, shooterVelocity.getVelocityFilter()/6000);
        shooter.setPower(x);
        shooter2.setPower(-x);
    }
    public void noPhysShootLiorPID(double x){
        pid.setWanted(x);
        pid.setWanted(x);
    }
    int count = 0;
    boolean prevMore = false;
    boolean prevLess = false;

    public double power = 0;
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
//        noPhysShootNext(power);
        PIDFCoefficients pidNew = new PIDFCoefficients(kP, kI, kD,kF);

        shooter.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidNew);
        shooter2.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidNew);

        shooter.setPower(power*errorFix);
        shooter2.setPower(-power*errorFix);
//        telemetry.addData("wanted variable", power*6000);
//        telemetry.addData("wanted fixed", errorFix*power*6000);
    }
    public void setDashBoardPID(){
        PIDFCoefficients pidNew = new PIDFCoefficients(kP, kI, kD,kF);
        shooter.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidNew);
        shooter2.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidNew);
    }


    public void interpolate(double dis){
        noPhysShoot((-9.90643e-11) * Math.pow(dis, 4) + ( 9.37832e-8 ) * Math.pow(dis, 3) - (2.81195e-5)  * Math.pow(dis, 2) + (3.61685e-3)  * dis + 0.315642);
    }

//    / @param dis: distance from goal
    /// shoots with different powers depending on what launch zone youre in
    // TODO: make depend on odo vals, closed loop control for values
    public void naiveShooter(boolean far) {
        PIDFCoefficients pidNew = new PIDFCoefficients(kP, kI, kD,kF);

        shooter.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidNew);
        shooter2.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidNew);
        if (!far) {
            Szonedis = 0.471;
        } else{
            Szonedis = 0.545;
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


    public void setShooterTelemetry(Telemetry telemetry){
        telemetry.addData("wanted", Szonedis*6000);
        telemetry.addData("vel",shooterVelocity.getVelocityFilter());
        telemetry.addData("vel2", shooter2Velocity.getVelocityFilter());
//        telemetry.addData("th",Math.abs(shooterVelocity.getVelocityFilter() - Szonedis*6000));
        telemetry.addData("pow 1", shooter.getPower());
        telemetry.addData("pow 2", shooter2.getPower());
        PIDFCoefficients coefficients = shooter2.getPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER);
        telemetry.addData("p", coefficients.p);
        telemetry.addData("i", coefficients.i);
        telemetry.addData("d", coefficients.d);
        telemetry.addData("wanted variable", power*6000);
        telemetry.addData("f",coefficients.f);
    }

    public boolean isUpToSpeed(){
        double threshold = 130; //TODO: tune!! should be the biggest reliably scoring value
        return Math.abs(shooterVelocity.getVelocityFilter() - Szonedis*6000) < threshold;
    }
    public boolean isUpToGivenSpeed(double speed){
        double threshold = 130; //TODO: tune!! should be the biggest reliably scoring value
        return Math.abs(shooterVelocity.getVelocityFilter() - speed*6000) < threshold;
    }
    public void out(){
        shooter.setPower(-.2);
        shooter2.setPower(.2);
    }
}