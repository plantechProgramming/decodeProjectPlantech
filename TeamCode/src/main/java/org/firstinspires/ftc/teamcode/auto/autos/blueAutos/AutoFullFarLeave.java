package org.firstinspires.ftc.teamcode.auto.autos.blueAutos; // make sure this aligns with class location
import static com.pedropathing.ivy.groups.Groups.sequential;
import static com.pedropathing.ivy.pedro.PedroCommands.follow;
import static dev.nextftc.extensions.pedro.PedroComponent.follower;

import com.pedropathing.ivy.Command;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.auto.AutoCommands;
import org.firstinspires.ftc.teamcode.auto.PathsBlue;
import org.firstinspires.ftc.teamcode.auto.autos.ReadWrite;
import org.firstinspires.ftc.teamcode.auto.pedro.constants.Constants;

import dev.nextftc.core.commands.delays.Delay;
import dev.nextftc.core.commands.groups.SequentialGroup;
import dev.nextftc.extensions.pedro.FollowPath;
import dev.nextftc.extensions.pedro.PedroComponent;
import dev.nextftc.ftc.NextFTCOpMode;

@Autonomous(name = "Full Far Blue leave", group = "Blue")
public class AutoFullFarLeave extends LinearOpMode {

//    public AutoFullFarLeave() {
//        addComponents(
////                new SubsystemComponent(NextShooter.INSTANCE, NextInBetween.INSTANCE),
//                new PedroComponent(Constants::createFollower),
//                AutoCommands.INSTANCE_BLUE
//        );
//    }
    AutoCommands command;
    PathsBlue path;
    ReadWrite readWrite = new ReadWrite();

        public Command autoRoutine(){
            return sequential(
                    command.startShooter(true),
//                    new Delay(0.4),
                    command.score(path.scorePreloadFar),
                    command.intake(path.grabGPPFar),
                    command.score(path.scoreGPPFar),

                    command.startShooter(false),
                    command.intake(path.grabPGPFar),
                    command.score(path.scorePGP),
                    follow(follower(), path.scoreLeaveFar)
//                    command.intake(path.intakePPG,path.grabPPG,0.75)
            );
        }
//    public void onStartButtonPressed() {
//
//    }
    public void onStop(){
        readWrite.writePose(follower().getPose());
    }

    @Override
    public void runOpMode() {
        path = new PathsBlue();
        follower().setStartingPose(path.getSPoseFar());
        path.buildPaths(follower());
        command = new AutoCommands(follower(), hardwareMap);
        autoRoutine();
    }
}