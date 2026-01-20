package org.firstinspires.ftc.teamcode.auto.subsystems;


import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.FtcDashboard;
import org.firstinspires.ftc.robotcore.external.Telemetry;

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
import dev.nextftc.hardware.impl.MotorEx;
import dev.nextftc.hardware.impl.ServoEx;
import dev.nextftc.hardware.positionable.SetPosition;
import dev.nextftc.hardware.powerable.SetPower;

public class NextShooter implements Subsystem {
    public static final NextShooter INSTANCE = new NextShooter();
    public NextShooter() {
    }
    FtcDashboard dashboard = FtcDashboard.getInstance();
    Telemetry dashboardTelemetry =  dashboard.getTelemetry();
    private MotorEx shooter1 = new MotorEx("shooter");
    private MotorEx shooter2 = new MotorEx("shooter2");


    double Szonedis = 0.5;
    double kp = 0.01, ki = 0, kd = 0, kf = 0;
    ControlSystem controlSystem = ControlSystem.builder() // next pid
            .velPid(kp, ki, kd)
            .basicFF(0,0,kf)
            .build();

    public Command naiveShooter(boolean far) {
        return new InstantCommand(
                () -> {
                    if (!far) {
                        Szonedis = 0.47;
                    } else {
                        Szonedis = 0.53;
                    }
                    controlSystem.setGoal(new KineticState(0, powerToTicks(Szonedis)));
                }
        );
    }
    double diameter = 0.096;
    double TICK_PER_REV = 28;
    double powerToTicks(double power){
        double rpm = power*6000;
        double rotPerSec = rpm/60;
        return TICK_PER_REV*rotPerSec; // rotations per second*amt of ticks per turn
    }
    double ticksToRPM(double ticks){
        double tickPerMin = ticks*60;
        return tickPerMin/TICK_PER_REV;
    }

    public void setTelemetry(Telemetry telemetry){
        telemetry.addData("wanted", ticksToRPM(controlSystem.getGoal().getVelocity()));
        telemetry.addData("vel",ticksToRPM(shooter1.getVelocity()));
        telemetry.addData("vel2", -ticksToRPM(shooter2.getVelocity()));
        telemetry.addData("measured pow", shooter1.getPower());
        telemetry.addData("goal", controlSystem.getGoal());
        telemetry.addData("szonedis", Szonedis);
        telemetry.update();
    }
    public void setPowerPID(MotorEx motor, MotorEx motor2){
        KineticState state = motor.getState();
        KineticState state2 = motor2.getState();
        motor.setPower(controlSystem.calculate(state));
        motor2.setPower(-controlSystem.calculate(state2));
        dashboardTelemetry.addData("pow", controlSystem.calculate(state));
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
        setPowerPID(shooter1, shooter2);
//        shooter1.setPower(Szonedis);
//        shooter2.setPower(-Szonedis);
        setTelemetry(dashboardTelemetry);
    }

    public void stop(){
        shooter2.setPower(0);
        shooter1.setPower(0);
    }
}
