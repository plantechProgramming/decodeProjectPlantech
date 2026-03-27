package org.firstinspires.ftc.teamcode.auto.autos.blueAutos;

import static dev.nextftc.extensions.pedro.PedroComponent.follower;

import com.pedropathing.follower.Follower;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.auto.AutoCommands;
import org.firstinspires.ftc.teamcode.auto.PathsBlue;
import org.firstinspires.ftc.teamcode.auto.autos.ReadWrite;
import org.firstinspires.ftc.teamcode.auto.pedro.constants.Constants;

import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.groups.SequentialGroup;
import dev.nextftc.extensions.pedro.FollowPath;
import dev.nextftc.extensions.pedro.PedroComponent;
import dev.nextftc.ftc.NextFTCOpMode;

@Autonomous(name = " OpenGateClose Blue", group = "Blue")
public class OpenGateClose extends NextFTCOpMode{

    private Follower follower;

    public OpenGateClose() {
        addComponents(
//                new SubsystemComponent(NextShooter.INSTANCE, NextInBetween.INSTANCE),
                new PedroComponent(Constants::createFollower),
                AutoCommands.INSTANCE_BLUE
        );
    }

    AutoCommands command = AutoCommands.INSTANCE_BLUE;
    PathsBlue path;
    ReadWrite readWrite = new ReadWrite();


    public Command autoRoutine(){
        return new SequentialGroup(
                command.startShooter(false),
                command.score(path.scorePreload),

                command.intake(path.grabPPG),
                command.score(path.scorePPG),

                command.intake(path.grabPGP),
                new FollowPath(path.gatePGP),

                command.score(path.scoreGateClose),
                new FollowPath(path.scoreLeaveClose)
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
        path = new PathsBlue();
        follower().setStartingPose(path.getSPose());
        path.buildPaths(follower());
        autoRoutine().schedule();
    }

    @Override
    public void onStop(){
        readWrite.writePose(follower().getPose());
    }
}
