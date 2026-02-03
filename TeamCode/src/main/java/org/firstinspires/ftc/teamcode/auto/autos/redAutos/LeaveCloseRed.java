package org.firstinspires.ftc.teamcode.auto.autos.redAutos;

import static dev.nextftc.extensions.pedro.PedroComponent.follower;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.auto.AutoCommands;
import org.firstinspires.ftc.teamcode.auto.PathsBlue;
import org.firstinspires.ftc.teamcode.auto.PathsRed;
import org.firstinspires.ftc.teamcode.auto.autos.ReadWrite;
import org.firstinspires.ftc.teamcode.auto.pedro.constants.Constants;

import dev.nextftc.core.commands.Command;
import dev.nextftc.extensions.pedro.FollowPath;
import dev.nextftc.extensions.pedro.PedroComponent;
import dev.nextftc.ftc.NextFTCOpMode;

@Autonomous(name = "Leave Close Red",group = "Red")
public class LeaveCloseRed extends NextFTCOpMode {
    private Follower follower;

    public LeaveCloseRed() {
        addComponents(
//                new SubsystemComponent(NextShooter.INSTANCE, NextInBetween.INSTANCE),
                new PedroComponent(Constants::createFollower),
                AutoCommands.INSTANCE
        );
    }

    AutoCommands command = AutoCommands.INSTANCE;
    PathsRed path;
    ReadWrite readWrite = new ReadWrite();

    private final Pose startPose = new Pose(20.1, 122.5, Math.toRadians(144)); // Start Pose of our robot.


    public Command autoRoutine() {
        return new FollowPath(path.leaveClose);
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
        follower().setStartingPose(path.getSPose());
        path.buildPaths(follower());
        autoRoutine().schedule();
    }
}
