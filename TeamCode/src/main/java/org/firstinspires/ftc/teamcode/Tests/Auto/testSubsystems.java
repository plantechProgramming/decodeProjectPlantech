package org.firstinspires.ftc.teamcode.Tests.Auto;

import com.pedropathing.ivy.Command;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.pedropathing.ivy.Scheduler;

import static com.pedropathing.ivy.Scheduler.schedule;
import static com.pedropathing.ivy.groups.Groups.*;
import static com.pedropathing.ivy.pedro.PedroCommands.follow;

import org.firstinspires.ftc.teamcode.subsystems.AutoCommands;

@Autonomous(group = "tests")
public class testSubsystems extends LinearOpMode {

    AutoCommands command;

    public Command autoRoutine(){
        return sequential(

        );
    }

    @Override
    public void runOpMode() {

        Scheduler.schedule(autoRoutine());

        while (opModeIsActive()) {
            command.periodic();
            Scheduler.execute();
        }
    }
}