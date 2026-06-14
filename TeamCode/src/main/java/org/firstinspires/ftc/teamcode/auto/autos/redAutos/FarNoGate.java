package org.firstinspires.ftc.teamcode.auto.autos.redAutos;

import static dev.nextftc.extensions.pedro.PedroComponent.follower;

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

@Autonomous(name = "Far with no gate red", group = "Red")
public class FarNoGate extends NextFTCOpMode {
    AutoCommands command;
    PathsRed path;
    ReadWrite readWrite = new ReadWrite();
    public FarNoGate(){
        addComponents(
//                new SubsystemComponent(NextShooter.INSTANCE, NextInBetween.INSTANCE),
                new PedroComponent(Constants::createFollower)
        );
    }

    public Command autoRoutine(){
        return new SequentialGroup(
                command.startShooter(true),
//                    new Delay(0.4),
                command.score(path.scorePreloadFar),
                command.intake(path.grabGPPFar),
                command.score(path.scoreGPPFar),

                command.startShooter(false),
                command.intake(path.grabPGPFar),
                command.score(path.scorePGP),
                new FollowPath(path.scoreLeaveFar)
//                    command.intake(path.intakePPG,path.grabPPG,0.75)
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
        follower().setStartingPose(path.getSPoseFar());
        path.buildPaths(follower());
        autoRoutine().schedule();
    }
}
