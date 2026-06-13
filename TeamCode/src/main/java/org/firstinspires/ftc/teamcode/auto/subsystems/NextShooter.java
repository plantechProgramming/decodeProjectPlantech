package org.firstinspires.ftc.teamcode.auto.subsystems;


import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.teleOp.PID;
import org.firstinspires.ftc.teamcode.teleOp.Utils;
import org.firstinspires.ftc.teamcode.teleOp.actions.GetVelocity;
import org.firstinspires.ftc.teamcode.teleOp.actions.Shooter;

import dev.nextftc.control.ControlSystem;
import dev.nextftc.control.KineticState;
import dev.nextftc.control.feedforward.FeedforwardElement;
import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.groups.ParallelDeadlineGroup;
import dev.nextftc.core.commands.groups.ParallelGroup;
import dev.nextftc.core.commands.groups.SequentialGroup;
import dev.nextftc.core.commands.utility.InstantCommand;
import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.core.units.Distance;
import dev.nextftc.ftc.NextFTCOpMode;
import dev.nextftc.hardware.delegates.Caching;
import dev.nextftc.hardware.impl.MotorEx;
import dev.nextftc.hardware.impl.ServoEx;
import dev.nextftc.hardware.positionable.SetPosition;
import dev.nextftc.hardware.powerable.SetPower;
@Configurable
@Config
public class NextShooter implements Subsystem {
//    public static final NextShooter INSTANCE = new NextShooter();
    VoltageSensor voltageSensor;
    public NextShooter(VoltageSensor voltageSensor) {
        this.voltageSensor = voltageSensor;
    }
    FtcDashboard dashboard = FtcDashboard.getInstance();
    Telemetry dashboardTelemetry =  dashboard.getTelemetry();
    private MotorEx shooter1 = new MotorEx("ShooterClose", -1);
    private MotorEx shooter2 = new MotorEx("ShooterFar", -1);
    double Szonedis = 0.5;
    public static double farPow = 0.5312;
    public static double closePow = 0.3905;
//    public static double kp = 0, ki = 0, kd = 0, kf = 0.0000011, ks = 0.1;
    public static double kp = Shooter.kP, ki = Shooter.kI, kd = Shooter.kD, kf = Shooter.kF, ks = Shooter.kS;
//    ControlSystem controlSystem = ControlSystem.builder() // next pid
//            .velPid(kp, ki, kd)
//            .basicFF(kf,0,ks)
//            .build();
    PID pid = new PID(kp, ki, kd, kf, ks);

    Utils utils = new Utils();

    public Command naiveShooter(boolean far) {
        return new InstantCommand(
                () -> {
                    if (!far) {
                        Szonedis = closePow;
                    } else {
                        Szonedis = farPow;
                    }
//                    controlSystem.setGoal(new KineticState(0, powerToTicks(Szonedis)));
                    pid.setWanted(Szonedis);
                }
        );
    }
    double diameter = 0.096;
    double TICK_PER_REV = 8192;
    double powerToTicks(double power){
        double rpm = power*6000;
        double rotPerSec = rpm/60;
        return TICK_PER_REV*rotPerSec; // rotations per second*amt of ticks per turn
    }
    double ticksToRPM(double ticks){
        double tickPerMin = ticks*60;
        return tickPerMin/TICK_PER_REV;
    }

    double RPMtoTicks(double RPM){
        double tickPerMin = RPM*TICK_PER_REV;
        return tickPerMin/60;
    }

    public void setTelemetry(Telemetry telemetry, double currVel){
//        telemetry.addData("wanted", ticksToRPM(controlSystem.getGoal().getVelocity()));
        telemetry.addData("vel", currVel);
        telemetry.addData("measured pow", shooter1.getPower());
//        telemetry.addData("goal", controlSystem.getGoal());
        telemetry.addData("wanted", Szonedis*6000);
        telemetry.update();
    }
//    public void setPowerPID(MotorEx motor, MotorEx motor2){
//        KineticState state = new KineticState(motor.getCurrentPosition(), RPMtoTicks(getRawVelocity()));
//        double pow = controlSystem.calculate(state);
//        motor.setPower(pow);
//        motor2.setPower(-pow);
//        dashboardTelemetry.addData("pow", pow);
//        dashboardTelemetry.addData("motor state vel", ticksToRPM(state.getVelocity()));
//    }
    public void setPowerPID(MotorEx motor, MotorEx motor2, double currVel){
        double pow = pid.update(currVel/6000);
        double voltageCompensated = utils.getVoltageCompensatedPow(pow, voltageSensor.getVoltage());
        motor.setPower(voltageCompensated);
        motor2.setPower(-voltageCompensated);
        dashboardTelemetry.addData("pow", pow);
////        dashboardTelemetry.addData("motor state vel", ticksToRPM(state.getVelocity()));
    }

    public Command setPow(){
        return new ParallelGroup(
                new SetPower(shooter1, Szonedis),
                new SetPower(shooter2, -Szonedis)
        );
    }


    // NOT subsystem periodic func, with normal periodic it initialized szonedis to 1 every second
    // loop
    public void Periodic(){
        double vel = getRawVelocity();
        setPowerPID(shooter1, shooter2, vel);
//        shooter1.setPower(Szonedis);
//        shooter2.setPower(-Szonedis);
        setTelemetry(dashboardTelemetry, vel);
    }

    public void stop(){
        shooter2.setPower(0);
        shooter1.setPower(0);
    }
    public long prevEncoder = 0;
    public long curEncoder;
    public double prevTime = 0;
    public double curTime;
    int millisecondsToMinute = 60000;
    int ticksPerRevolution = 8192; // 8192 for thru bore, 28 for built in
    private static final ElapsedTime timer = new ElapsedTime();
    public double getRawVelocity() {
        curEncoder = (long)shooter1.getCurrentPosition();
        curTime = timer.milliseconds();

        double timeDiff = curTime - prevTime;
        double encoderDiff = curEncoder - prevEncoder;

        double tickVelocity = encoderDiff / timeDiff; // ticks/milliseconds
        double curVelocity = (tickVelocity * millisecondsToMinute) / ticksPerRevolution;
        prevEncoder = curEncoder;
        prevTime = curTime;
        return curVelocity;
    }

    double prevVelocity = 0;
    public double getVelocityFilter() {
        double velocity = getRawVelocity();
        double filteredVelocity = utils.filter(0.1, velocity, prevVelocity);
        prevVelocity = filteredVelocity;
        return filteredVelocity;
    }
}
