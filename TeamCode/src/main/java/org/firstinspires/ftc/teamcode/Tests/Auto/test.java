package org.firstinspires.ftc.teamcode.Tests.Auto;

import static com.pedropathing.ivy.groups.Groups.sequential;
import static com.pedropathing.ivy.pedro.PedroCommands.follow;

import com.pedropathing.follower.Follower;
import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.Scheduler;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Misc.Utils.Alliance;
import org.firstinspires.ftc.teamcode.auto.autos.paths.Paths;
import org.firstinspires.ftc.teamcode.auto.autos.paths.Points;
import org.firstinspires.ftc.teamcode.auto.pedro.Constants;
import org.firstinspires.ftc.teamcode.subsystems.AutoCommands;

@Disabled
@Autonomous(group="tests")
public class test extends LinearOpMode {


    AutoCommands command;
    Paths path;
    Follower follower;

    public Command autoRoutine(){
        return sequential(

        );
    }

    @Override
    public void runOpMode() {
        Alliance.set(Alliance.BLUE);
        path = new Paths();
        follower = Constants.createFollower(hardwareMap);

        follower.setStartingPose(Points.startPoseFar);
        command = new AutoCommands(follower);
        path.buildPaths(follower);

        Scheduler.schedule(autoRoutine());

        while (opModeIsActive()) {
            command.periodic();
            Scheduler.execute();
        }
    }

}
