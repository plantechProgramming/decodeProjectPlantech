package org.firstinspires.ftc.teamcode.Tests.TeleOp;

import com.acmerobotics.dashboard.config.Config;
import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.Misc.Alliance;
import org.firstinspires.ftc.teamcode.TeamOpMode;
import org.firstinspires.ftc.teamcode.Misc.Txt.ReadWrite;
import org.firstinspires.ftc.teamcode.subsystems.AutoCommands;
import org.firstinspires.ftc.teamcode.subsystems.DriveTrain;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;

@Configurable
@Config
@TeleOp(group = "tests")
public class shooterTest extends TeamOpMode {

    @Override
    protected void run() {
        Shooter shooter = new Shooter();
        AutoCommands commands = new AutoCommands();
        while(opModeIsActive()){
            shooter.variableShoot(gamepad1.dpad_up, gamepad1.dpad_down, .01);

            if(gamepad1.a){
                commands.shoot();
            }

            shooter.updateTelemetry(dashboardTelemetry);
            shooter.updateTelemetry(telemetry);
            telemetry.update();
            dashboardTelemetry.update();
        }
    }

    @Override
    protected void end() {

    }
}
