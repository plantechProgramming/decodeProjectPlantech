package org.firstinspires.ftc.teamcode.auto.autos.redAutos;

import static dev.nextftc.extensions.pedro.PedroComponent.follower;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.auto.AutoCommands;
import org.firstinspires.ftc.teamcode.auto.PathsRed;
import org.firstinspires.ftc.teamcode.auto.autos.ReadWrite;
import org.firstinspires.ftc.teamcode.auto.pedro.constants.Constants;

import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.delays.Delay;
import dev.nextftc.core.commands.groups.SequentialGroup;
import dev.nextftc.extensions.pedro.FollowPath;
import dev.nextftc.extensions.pedro.PedroComponent;
import dev.nextftc.ftc.NextFTCOpMode;

@Autonomous(name = "Leave Far Pre Red",group = "Red")
public class LeaveFarPre extends NextFTCOpMode {
    private Follower follower;

    public LeaveFarPre() {
        addComponents(
//                new SubsystemComponent(NextShooter.INSTANCE, NextInBetween.INSTANCE),
                new PedroComponent(Constants::createFollower),
                AutoCommands.INSTANCE_RED
        );
    }

    AutoCommands command = AutoCommands.INSTANCE_RED;
    PathsRed path;
    ReadWrite readWrite = new ReadWrite();

    private final Pose startPose = new Pose(56.15, 8.4, Math.toRadians(90)); // Start Pose of our robot.

    public Command autoRoutine(){
        return new SequentialGroup(
                command.startShooter(true),
                new Delay(1),
                command.score(path.scorePreloadFar),
                new FollowPath(path.scoreLeaveFar)
        );
    }

    @Override
    public void onUpdate(){
        telemetry.addData("x", follower().getPose().getX());
        telemetry.addData("y", follower().getPose().getY());
        telemetry.addData("heading", follower().getPose().getHeading());
        telemetry.update();
    }
    @Override
    public void onStartButtonPressed() {
        path = new PathsRed();
        follower().setStartingPose(path.getSPoseFar());
        path.buildPaths(follower());
        autoRoutine().schedule();
    }
    @Override
    public void onStop(){
        ReadWrite readWrite = new ReadWrite();
        readWrite.writePose(follower().getPose());
    }
}
