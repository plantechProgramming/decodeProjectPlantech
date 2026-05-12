package org.firstinspires.ftc.teamcode.Tests.Auto;

import com.acmerobotics.dashboard.FtcDashboard;
import com.pedropathing.ivy.Command;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.pedropathing.ivy.Scheduler;

import static com.pedropathing.ivy.Scheduler.schedule;
import static com.pedropathing.ivy.groups.Groups.*;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.subsystems.InBetween;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;

//@Autonomous(name="testNext", group="test")
@TeleOp
public class testNext extends LinearOpMode {

    Telemetry dashboardTelemetry;
    @Override
    public void runOpMode() {
        FtcDashboard dashboard = FtcDashboard.getInstance();
        dashboardTelemetry = dashboard.getTelemetry();

        Intake intake = new Intake(hardwareMap);
        Shooter shooter = new Shooter(hardwareMap);
        InBetween inBetween = new InBetween(hardwareMap);

        Command test = parallel(
                intake.take(),
                shooter.naiveShooter(false),
                inBetween.inBetweenInFull()
        );
        Scheduler.reset();
        waitForStart();
        Scheduler.schedule(test);
        // Schedule the sequence when the OpMode starts
        while (opModeIsActive()) {
            // Run the scheduler each loop
            shooter.periodic(); //TODO: make this work when scheduled
            shooter.updateTelemetry(telemetry);
            shooter.updateTelemetry(dashboardTelemetry);
            Scheduler.execute();
        }
    }
}