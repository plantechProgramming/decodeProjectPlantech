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
import dev.nextftc.extensions.pedro.FollowPath;
import dev.nextftc.extensions.pedro.PedroComponent;
import dev.nextftc.ftc.NextFTCOpMode;

@Autonomous(name = "Megido Auto Close Red", group = "Red")
public class MegidoAuto extends NextFTCOpMode {
    AutoCommands command;

    public MegidoAuto() {
        addComponents(
//                new SubsystemComponent(NextShooter.INSTANCE, NextInBetween.INSTANCE),
                new PedroComponent(Constants::createFollower)
        );
    }

    PathsRed path;
    ReadWrite readWrite = new ReadWrite();

    public Command autoRoutine(){
        return new SequentialGroup(
                command.startShooter(false),
                command.scorePreload(path.scorePreload),
                command.intake(path.grabPPG),

                command.score(path.scorePPG),
                command.intake(path.grabPGP),

                command.score(path.scorePGP),
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
        path = new PathsRed();
        follower().setStartingPose(path.getSPose());
        path.buildPaths(follower());
        autoRoutine().schedule();
    }

}
