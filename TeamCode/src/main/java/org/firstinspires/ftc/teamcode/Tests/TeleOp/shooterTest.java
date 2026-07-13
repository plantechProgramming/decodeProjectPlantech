package org.firstinspires.ftc.teamcode.Tests.TeleOp;

import static com.pedropathing.ivy.Scheduler.schedule;

import com.acmerobotics.dashboard.config.Config;
import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.ivy.Scheduler;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Misc.InitMotors;
import org.firstinspires.ftc.teamcode.Misc.Utils.TelemetryUtils;
import org.firstinspires.ftc.teamcode.TeamOpMode;
import org.firstinspires.ftc.teamcode.subsystems.AutoCommands;
import org.firstinspires.ftc.teamcode.subsystems.InBetween;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;

@Configurable
@Config
@TeleOp(group = "telOp tests")
public class shooterTest extends TeamOpMode {

    @Override
    protected void run() {
        Shooter shooter = new Shooter();
        Intake intake = new Intake();
        AutoCommands commands = new AutoCommands();
        while(opModeIsActive()){
            shooter.variableShoot(gamepad1.dpad_up, gamepad1.dpad_down, .01);

            if(gamepad1.a){
                schedule(commands.shoot());
            }
            shooter.updateTelemetry(dashboardTelemetry);
            shooter.updateTelemetry(telemetry);
            telemetry.update();
            dashboardTelemetry.update();
            shooter.periodic();
            Scheduler.execute();
        }
    }

    @Override
    protected void end() {

    }
}
