package org.firstinspires.ftc.teamcode.auto.autos.blueAutos; // make sure this aligns with class location
import static dev.nextftc.extensions.pedro.PedroComponent.follower;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.auto.AutoCommands;
import org.firstinspires.ftc.teamcode.auto.PathsBlue;
import org.firstinspires.ftc.teamcode.auto.autos.ReadWrite;
import org.firstinspires.ftc.teamcode.auto.pedro.constants.Constants;

import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.delays.Delay;
import dev.nextftc.core.commands.groups.SequentialGroup;
import dev.nextftc.extensions.pedro.PedroComponent;
import dev.nextftc.ftc.NextFTCOpMode;

@Autonomous(name = "FullFarBlue")
public class AutoFullFar extends NextFTCOpMode {

    public AutoFullFar() {
        addComponents(
//                new SubsystemComponent(NextShooter.INSTANCE, NextInBetween.INSTANCE),
                new PedroComponent(Constants::createFollower),
                AutoCommands.INSTANCE
        );
    }

    AutoCommands command = AutoCommands.INSTANCE;
    PathsBlue path;
    ReadWrite readWrite = new ReadWrite();

        public Command autoRoutine(){
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
        path = new PathsBlue();
        follower().setStartingPose(path.getSPoseFar());
        path.buildPaths(follower());
        autoRoutine().schedule();
    }
    @Override
    public void onStop(){
        readWrite.writePose(follower().getPose());
    }
}