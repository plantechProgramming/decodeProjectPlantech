package org.firstinspires.ftc.teamcode.auto.autos.redAutos;

import static dev.nextftc.extensions.pedro.PedroComponent.follower;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.auto.AutoCommands;
import org.firstinspires.ftc.teamcode.auto.PathsBlue;
import org.firstinspires.ftc.teamcode.auto.PathsRed;
import org.firstinspires.ftc.teamcode.auto.pedro.constants.Constants;

import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.delays.Delay;
import dev.nextftc.core.commands.groups.SequentialGroup;
import dev.nextftc.extensions.pedro.PedroComponent;
import dev.nextftc.ftc.NextFTCOpMode;


@Autonomous(name = "FullFarRed")
public class FullAutoFar extends NextFTCOpMode {
    private Follower follower;

    public FullAutoFar() {
        addComponents(
//                new SubsystemComponent(NextShooter.INSTANCE, NextInBetween.INSTANCE),
                new PedroComponent(Constants::createFollower),
                AutoCommands.INSTANCE
        );
    }

    AutoCommands command = AutoCommands.INSTANCE;
    PathsRed path;


    private final Pose startPose = new Pose(62, 8, Math.toRadians(90)).mirror(); // Start Pose of our robot.

    public Command autoRoutine(){
        path.buildPaths();
        return new SequentialGroup(
                command.startShooter(true),
                new Delay(0.3),
                command.score(path.scorePreloadFar),
                command.intake(path.intakeGPP, path.grabGPPFar, 0.73),
                command.score(path.scoreGPPFar),

                command.startShooter(false),
                command.intake(path.intakePGP,path.grabPGPFar,0.72),
                command.score(path.scorePGP),
                command.intake(path.intakePPG,path.grabPPG,0.72)

        );

    }
    @Override
    public void onStartButtonPressed() {
        follower().setStartingPose(startPose);
        path = new PathsRed(follower());
        path.buildPaths();
        autoRoutine().schedule();
    }
}
