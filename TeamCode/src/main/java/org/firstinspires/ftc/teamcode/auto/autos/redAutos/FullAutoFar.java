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
import dev.nextftc.core.commands.delays.Delay;
import dev.nextftc.core.commands.groups.SequentialGroup;
import dev.nextftc.extensions.pedro.PedroComponent;
import dev.nextftc.ftc.NextFTCOpMode;


@Autonomous(name = "Full Far Red", group = "Red")
public class FullAutoFar extends NextFTCOpMode {

    public FullAutoFar() {
        addComponents(
                new PedroComponent(Constants::createFollower),
                AutoCommands.INSTANCE_RED
        );
    }

    AutoCommands command = AutoCommands.INSTANCE_RED;
    PathsRed path;


    private final Pose startPose = new Pose(56.15, 8.4, Math.toRadians(90)).mirror(); // Start Pose of our robot.

    public Command autoRoutine(){
        return new SequentialGroup(
                command.startShooter(true),
                new Delay(0.3),
                command.score(path.scorePreloadFar),
                command.intake(path.intakeGPP, path.grabGPPFar, 0.72),
                command.score(path.scoreGPPFar),

                command.startShooter(false),
                command.intake(path.intakePGP,path.grabPGPFar,0.72),
                command.score(path.scorePGP),
                command.intake(path.intakePPG,path.grabPPG,0.72)

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
//        telemetry.addData("start x", follower().getPose().getX());
//        telemetry.addData("start y", follower().getPose().getY());
//        telemetry.addData("start heading", follower().getPose().getHeading());
//        telemetry.update();
        autoRoutine().schedule();
    }


    @Override
    public void onStop(){
        ReadWrite readWrite = new ReadWrite();
        readWrite.writePose(follower().getPose());
    }
}
