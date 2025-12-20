package org.firstinspires.ftc.teamcode.auto.test;


import static dev.nextftc.extensions.pedro.PedroComponent.follower;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.auto.AutoCommands;
import org.firstinspires.ftc.teamcode.auto.pedro.constants.Constants;
import org.firstinspires.ftc.teamcode.auto.subsystems.NextInBetween;
import org.firstinspires.ftc.teamcode.auto.subsystems.NextIntake;
import org.firstinspires.ftc.teamcode.auto.subsystems.NextShooter;

import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.groups.SequentialGroup;
import dev.nextftc.core.components.SubsystemComponent;
import dev.nextftc.extensions.pedro.FollowPath;
import dev.nextftc.extensions.pedro.PedroComponent;
import dev.nextftc.ftc.NextFTCOpMode;
import dev.nextftc.ftc.components.BulkReadComponent;
import dev.nextftc.core.commands.CommandManager;

@Autonomous(name = "NextFTC testing",group = "tests")
public class testNext extends NextFTCOpMode {

AutoCommands command = new AutoCommands();
    public testNext() {
        addComponents(
                new SubsystemComponent(NextShooter.INSTANCE, NextInBetween.INSTANCE),
                BulkReadComponent.INSTANCE,
                new PedroComponent(Constants::createFollower)
        );

    }

    private final Pose startPose = new Pose(19, 121.5, Math.toRadians(144)); // Start Pose of our robot.
    private final Pose endPose = new Pose(55.5, 61, Math.toRadians(180));
    private final Pose controlPose = new Pose(70,89);

    private PathChain leavePath;
    private Follower follower;


    public void buildPaths() {
        /* This is our scorePreload path. We are using a BezierLine, which is a straight line. */
        leavePath = follower().pathBuilder()
                .addPath(new BezierCurve(startPose,controlPose,endPose))
                .setLinearHeadingInterpolation(startPose.getHeading(),endPose.getHeading())
                .build();
    }

    private Command autonomousRoutine() {
        return new SequentialGroup(
                command.preload1(),
                new FollowPath(leavePath)
                );
    }
    @Override
    public void onStartButtonPressed() {
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startPose);
        buildPaths();
        autonomousRoutine().schedule();
    }

}