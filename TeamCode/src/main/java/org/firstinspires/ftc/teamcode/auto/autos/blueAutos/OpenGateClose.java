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

@Autonomous(name = " OpenGateClose Blue", group = "Blue")
public class OpenGateClose extends NextFTCOpMode{

    AutoCommands command;
    public OpenGateClose() {
        addComponents(
            new PedroComponent(Constants::createFollower)
        );

    }

    PathsBlue path;
    ReadWrite readWrite = new ReadWrite();


    public Command autoRoutine(){
        return new SequentialGroup(
                command.startShooter(false),
                command.scorePreload(path.scorePreload),

                command.intake(path.grabPGP),
                command.score(path.PGPandOpen),
                command.score(path.scorePGP),

                command.intake(path.pickUpGateFromScore),
                command.take(),
                new Delay(1),
                command.stopAll(),
                command.score(path.scorePickUpGate),

                command.intake(path.grabPPG),
                command.score(path.leavePPGClose)
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
