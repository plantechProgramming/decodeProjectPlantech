package org.firstinspires.ftc.teamcode.auto.test;

import static org.firstinspires.ftc.teamcode.auto.pedro.Tuning.follower;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.auto.AutoCommands;
import org.firstinspires.ftc.teamcode.auto.PathsBlue;
import org.firstinspires.ftc.teamcode.auto.PathsRed;
import org.firstinspires.ftc.teamcode.auto.autos.ReadWrite;
import org.firstinspires.ftc.teamcode.auto.pedro.constants.Constants;

import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.delays.Delay;
import dev.nextftc.core.commands.groups.SequentialGroup;
import dev.nextftc.extensions.pedro.FollowPath;
import dev.nextftc.extensions.pedro.PedroComponent;
import dev.nextftc.ftc.NextFTCOpMode;

@Autonomous(name="test",group="test")
public class test extends NextFTCOpMode {
    AutoCommands command;
    PathsBlue path;
    ReadWrite readWrite = new ReadWrite();
    public test(){
        addComponents(
                new PedroComponent(Constants::createFollower)
        );
    }

    public Command autoRoutine(){
        return new SequentialGroup(
                command.intake(path.pickUpGateFromFarStart)
//
//                    command.intakeWithSpeed(path.grabLeftoverBallsGate, 0.7),
//                    command.score(path.scoreLeftoverBallsGate),

        );
    }
    @Override
    public void onUpdate(){
        telemetry.addData("x", PedroComponent.follower().getPose().getX());
        telemetry.addData("y", PedroComponent.follower().getPose().getY());
        telemetry.addData("heading", PedroComponent.follower().getPose().getHeading());
        telemetry.update();
    }
    @Override
    public void onStartButtonPressed() {
        command = new AutoCommands(PedroComponent.follower(), hardwareMap.voltageSensor.iterator().next());
        addComponents(
                command
        );
        path = new PathsBlue();
        PedroComponent.follower().setStartingPose(path.getSPoseFar());
        path.buildPaths(PedroComponent.follower());
        autoRoutine().schedule();
    }

    @Override
    public void onStop(){
        readWrite.writePose(PedroComponent.follower().getPose());
    }
}
