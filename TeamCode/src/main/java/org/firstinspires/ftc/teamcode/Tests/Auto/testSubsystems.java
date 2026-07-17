package org.firstinspires.ftc.teamcode.Tests.Auto;

import com.pedropathing.ivy.Command;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.pedropathing.ivy.Scheduler;

import static com.pedropathing.ivy.Scheduler.schedule;
import static com.pedropathing.ivy.groups.Groups.*;
import static com.pedropathing.ivy.pedro.PedroCommands.follow;

import org.firstinspires.ftc.teamcode.subsystems.AutoCommands;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;

@Autonomous(group = "autonomous tests")
public class testSubsystems extends LinearOpMode {

    AutoCommands command;
    Shooter shooter;

    public Command autoRoutine(){
        return sequential(
            shooter.setPowerAsCommand(0.1)
        );
    }

    @Override
    public void runOpMode() {
        command = new AutoCommands();
        shooter = new Shooter();
        Scheduler.schedule(autoRoutine());
        while (opModeIsActive()) {
            command.periodic();
            Scheduler.execute();
        }
    }
}