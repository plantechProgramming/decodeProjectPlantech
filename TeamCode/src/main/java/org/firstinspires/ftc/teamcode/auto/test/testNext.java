package org.firstinspires.ftc.teamcode.auto.test;

import com.acmerobotics.dashboard.FtcDashboard;
import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.commands.Commands;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.pedropathing.ivy.Scheduler;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import static com.pedropathing.ivy.Scheduler.schedule;
import static com.pedropathing.ivy.commands.Commands.*;
import static com.pedropathing.ivy.groups.Groups.*;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.auto.subsystems.NextInBetween;
import org.firstinspires.ftc.teamcode.auto.subsystems.NextIntake;
import org.firstinspires.ftc.teamcode.auto.subsystems.NextShooter;

//@Autonomous(name="testNext", group="test")
@TeleOp
public class testNext extends LinearOpMode {
    Telemetry dashboardTelemetry;
    @Override
    public void runOpMode() {
        Scheduler.reset();
        FtcDashboard dashboard = FtcDashboard.getInstance();
        dashboardTelemetry = dashboard.getTelemetry();

        NextIntake intake = new NextIntake(hardwareMap);
        NextShooter shooter = new NextShooter(hardwareMap);
        NextInBetween inBetween = new NextInBetween(hardwareMap);

        Command test = parallel(
                intake.take(),
                shooter.naiveShooter(false),
                inBetween.inBetweenInFull()
        );

        waitForStart();

        Scheduler.schedule(test);
        // Schedule the sequence when the OpMode starts
        while (opModeIsActive()) {
            // Run the scheduler each loop
            shooter.periodic();
            shooter.updateTelemetry(telemetry);
            shooter.updateTelemetry(dashboardTelemetry);
            Scheduler.execute();
        }
    }
}