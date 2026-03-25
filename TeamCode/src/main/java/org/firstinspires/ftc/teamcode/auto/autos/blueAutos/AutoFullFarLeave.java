package org.firstinspires.ftc.teamcode.auto.autos.blueAutos; // make sure this aligns with class location
import static dev.nextftc.extensions.pedro.PedroComponent.follower;

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

@Autonomous(name = "Full Far Blue leave", group = "Blue")
public class AutoFullFarLeave extends NextFTCOpMode {

    public AutoFullFarLeave() {
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
                    command.startShooter(true),
//                    new Delay(0.4),
                    command.score(path.scorePreloadFar),
                    command.intake(path.intakeGPP, path.grabGPPFar),
                    command.score(path.scoreGPPFar),

                    command.startShooter(false),
                    command.intake(path.intakePGP,path.grabPGPFar),
                    command.score(path.scorePGP),
                    new FollowPath(path.scoreLeaveFar)
//                    command.intake(path.intakePPG,path.grabPPG,0.75)
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