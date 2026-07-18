package org.firstinspires.ftc.teamcode.Tests.Auto;

import static com.pedropathing.ivy.commands.Commands.waitMs;
import static com.pedropathing.ivy.groups.Groups.sequential;
import static com.pedropathing.ivy.pedro.PedroCommands.follow;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.Scheduler;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Misc.DataSaving;
import org.firstinspires.ftc.teamcode.Misc.Utils.Alliance;
import org.firstinspires.ftc.teamcode.Misc.Utils.TelemetryUtils;
import org.firstinspires.ftc.teamcode.TeamOpMode;
import org.firstinspires.ftc.teamcode.auto.TeamAuto;
import org.firstinspires.ftc.teamcode.auto.autos.paths.Paths;
import org.firstinspires.ftc.teamcode.auto.autos.paths.Points;
import org.firstinspires.ftc.teamcode.auto.pedro.Constants;
import org.firstinspires.ftc.teamcode.subsystems.AutoCommands;

@Autonomous(group="autonomous tests")
public class test extends TeamAuto {

    @Override
    public void postInit(){
        isFar = false;
        Alliance.set(Alliance.RED);
    }

    @Override
    public Command autoRoutine() {
        return sequential(
                command.startShooter(false),
                command.score(path.scorePreload),
                command.intake(path.grabPPG),
                command.score(path.scorePPG)
        );
    }

}
