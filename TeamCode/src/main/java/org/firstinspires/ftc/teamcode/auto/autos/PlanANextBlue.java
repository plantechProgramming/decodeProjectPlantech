package org.firstinspires.ftc.teamcode.auto.autos;

import static dev.nextftc.extensions.pedro.PedroComponent.follower;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import org.firstinspires.ftc.teamcode.auto.PathsBlue;
import org.firstinspires.ftc.teamcode.auto.AutoCommands;
import org.firstinspires.ftc.teamcode.auto.pedro.constants.Constants;
import org.firstinspires.ftc.teamcode.auto.subsystems.NextInBetween;
import org.firstinspires.ftc.teamcode.auto.subsystems.NextShooter;
import org.firstinspires.ftc.teamcode.teleOp.actions.Shooter;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.delays.Delay;
import dev.nextftc.core.commands.groups.ParallelGroup;
import dev.nextftc.core.commands.groups.SequentialGroup;
import dev.nextftc.core.commands.utility.InstantCommand;
import dev.nextftc.core.components.SubsystemComponent;
import dev.nextftc.extensions.pedro.FollowPath;
import dev.nextftc.extensions.pedro.PedroComponent;
import dev.nextftc.ftc.NextFTCOpMode;
import dev.nextftc.ftc.components.BulkReadComponent;

@Autonomous(name = "Plan A Next blue")
public class PlanANextBlue extends NextFTCOpMode{

    private Follower follower;

    public PlanANextBlue() {
        addComponents(
                new SubsystemComponent(NextShooter.INSTANCE, NextInBetween.INSTANCE),
                BulkReadComponent.INSTANCE,
                new PedroComponent(Constants::createFollower)
        );

    }

    AutoCommands command;
    PathsBlue path;
    public final Pose startPose = new Pose(19, 121.5, Math.toRadians(144)); // Start Pose of our robot.


    public Command autoRoutine(){
        path.buildPaths();
        return new SequentialGroup(
                command.startShooter(false),
                command.score(path.scorePreload),
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
        follower.setStartingPose(startPose);
        command = new AutoCommands(follower);
        path = new PathsBlue(follower);
        path.buildPaths();
        autoRoutine().schedule();

    }

}
