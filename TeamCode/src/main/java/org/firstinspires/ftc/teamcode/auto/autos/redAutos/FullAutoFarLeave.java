package org.firstinspires.ftc.teamcode.auto.autos.redAutos;

import static dev.nextftc.extensions.pedro.PedroComponent.follower;

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


@Autonomous(name = "Full Far Red leave", group = "Red")
public class FullAutoFarLeave extends NextFTCOpMode {

    AutoCommands command;
    public FullAutoFarLeave() {
        addComponents(
                new PedroComponent(Constants::createFollower)
        );
    }

    PathsRed path;


    public Command autoRoutine(){
        return new SequentialGroup(
                command.startShooter(true),
                command.scorePreload(path.scorePreloadFar),

                command.intake(path.grabGPPFar),
                command.score(path.scoreGPPFar),

                new Delay(3),
                command.intakeWithSpeed(path.grabLeftoverBallsGate, 0.7),
                command.score(path.scoreLeftoverBallsGate),
//
//                    command.intakeWithSpeed(path.grabLeftoverBallsGate, 0.7),
//                    command.score(path.scoreLeftoverBallsGate),

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
        command = new AutoCommands(follower(), hardwareMap.voltageSensor.iterator().next());
        addComponents(
                command
        );
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
