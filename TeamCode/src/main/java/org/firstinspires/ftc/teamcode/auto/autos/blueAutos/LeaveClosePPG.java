package org.firstinspires.ftc.teamcode.auto.autos.blueAutos;

import static dev.nextftc.extensions.pedro.PedroComponent.follower;

import com.pedropathing.follower.Follower;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.auto.AutoCommands;
import org.firstinspires.ftc.teamcode.auto.PathsBlue;
import org.firstinspires.ftc.teamcode.auto.autos.ReadWrite;
import org.firstinspires.ftc.teamcode.auto.pedro.constants.Constants;

import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.delays.Delay;
import dev.nextftc.core.commands.groups.SequentialGroup;
import dev.nextftc.extensions.pedro.FollowPath;
import dev.nextftc.extensions.pedro.PedroComponent;
import dev.nextftc.ftc.NextFTCOpMode;
@Autonomous(name = "Leave Close PPG Blue", group = "Blue")
public class LeaveClosePPG extends NextFTCOpMode {
    private Follower follower;

    public LeaveClosePPG() {
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
                new Delay(1),
                command.score(path.scorePreload),
                command.intake(path.intakePPG,path.grabPPG,0.5),
                command.score(path.scorePPG),
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
        ReadWrite readWrite = new ReadWrite();
        readWrite.writePose(follower().getPose());
    }
}
