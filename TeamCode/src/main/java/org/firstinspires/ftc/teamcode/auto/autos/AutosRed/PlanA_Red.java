package org.firstinspires.ftc.teamcode.auto.autos.AutosRed; // make sure this aligns with class location
import static dev.nextftc.extensions.pedro.PedroComponent.follower;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import  com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.auto.AutoCommands;
import org.firstinspires.ftc.teamcode.auto.PathsBlue;
import org.firstinspires.ftc.teamcode.auto.PathsRed;
import org.firstinspires.ftc.teamcode.auto.pedro.constants.Constants;

import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.groups.SequentialGroup;
import dev.nextftc.ftc.NextFTCOpMode;

@Autonomous(name = "PlanA_Red")
public class PlanA_Red extends NextFTCOpMode {
    private Timer pathTimer, actionTimer, opmodeTimer;
    private int pathState;

    AutoCommands command;
    PathsRed path;

    public Command autoRoutine(){
        path.buildPaths();
        return new SequentialGroup(
                command.startShooter(false),
                command.score(path.scorePreloadClose),
                command.intake(path.intakePPG,path.grabPPG,0.72),

                command.score(path.scorePPG),
                command.intake(path.intakePGP,path.grabPGP,0.72),

                command.score(path.scorePGP),
                command.intake(path.intakeGPP, path.grabGPP, 0.72)
        );
    }
    @Override
    public void onStartButtonPressed() {
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startPoseClose);
        command = new AutoCommands(follower);
        path = new PathsBlue(follower());
        path.buildPaths();
        autoRoutine().schedule();

    }
}