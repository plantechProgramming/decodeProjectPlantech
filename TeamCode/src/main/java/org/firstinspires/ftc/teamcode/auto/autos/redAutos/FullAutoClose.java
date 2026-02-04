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
import dev.nextftc.core.commands.groups.SequentialGroup;
import dev.nextftc.extensions.pedro.PedroComponent;
import dev.nextftc.ftc.NextFTCOpMode;

@Autonomous(name = "Full Close Red", group = "Red")
public class FullAutoClose extends NextFTCOpMode {
    private Follower follower;

    public FullAutoClose() {
        addComponents(
//                new SubsystemComponent(NextShooter.INSTANCE, NextInBetween.INSTANCE),
                new PedroComponent(Constants::createFollower),
                AutoCommands.INSTANCE_RED
        );
    }

    AutoCommands command = AutoCommands.INSTANCE_RED;
    PathsRed path;


    private final Pose startPose = new Pose(20.1, 122.5, Math.toRadians(144)).mirror(); // Start Pose of our robot.


    public Command autoRoutine(){
        return new SequentialGroup(
                command.startShooter(false),
                command.score(path.scorePreload),
                command.intake(path.intakePPG,path.grabPPG,0.64),

                command.score(path.scorePPG),
                command.intake(path.intakePGP,path.grabPGP,0.68),

                command.score(path.scorePGP),
                command.intake(path.intakeGPP, path.grabGPP, 0.67)
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
        follower().setStartingPose(path.getSPose());
        path.buildPaths(follower());
        autoRoutine().schedule();
    }

    @Override
    public void onStop(){
        ReadWrite readWrite = new ReadWrite();
        readWrite.writePose(follower().getPose());
    }
}
