package org.firstinspires.ftc.teamcode.auto.autos.blueAutos;

import static dev.nextftc.extensions.pedro.PedroComponent.follower;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;

import org.firstinspires.ftc.teamcode.auto.AutoCommands;
import org.firstinspires.ftc.teamcode.auto.PathsBlue;
import org.firstinspires.ftc.teamcode.auto.autos.ReadWrite;
import org.firstinspires.ftc.teamcode.auto.pedro.constants.Constants;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.delays.Delay;
import dev.nextftc.core.commands.groups.SequentialGroup;
import dev.nextftc.extensions.pedro.FollowPath;
import dev.nextftc.extensions.pedro.PedroComponent;
import dev.nextftc.ftc.NextFTCOpMode;

@Autonomous(name = "Leave Close Pre Blue",group = "Blue")
public class LeaveClosePre extends NextFTCOpMode{


    private Follower follower;
    AutoCommands command;
    public LeaveClosePre() {
        addComponents(
//                new SubsystemComponent(NextShooter.INSTANCE, NextInBetween.INSTANCE),
                new PedroComponent(Constants::createFollower)
        );
    }

    PathsBlue path;
    ReadWrite readWrite = new ReadWrite();

    private final Pose startPose = new Pose(20.1, 122.5, Math.toRadians(144)); // Start Pose of our robot.


    public Command autoRoutine(){
        return new SequentialGroup(
                command.startShooter(false),
                new Delay(1),
                command.scorePreload(path.scorePreload),
                new FollowPath(path.scoreLeaveClose)
        );
    }
    @Override
    public void onUpdate(){
        telemetry.addData("x", follower().getPose().getX());
        telemetry.addData("y", follower().getPose().getY());
        telemetry.addData("heading", follower().getPose().getHeading());
        telemetry.update();
        if(!(Math.round(follower().getPose().getY()) == 0 && Math.round(follower().getPose().getX()) == 0)){
            readWrite.writePose(follower().getPose());
        }
    }
    @Override
    public void onStartButtonPressed() {
        command = new AutoCommands(follower(), hardwareMap.voltageSensor.iterator().next());
        addComponents(
                command
        );
        path = new PathsBlue();
        follower().setStartingPose(path.getSPose());
        path.buildPaths(follower());
        autoRoutine().schedule();
    }
}

