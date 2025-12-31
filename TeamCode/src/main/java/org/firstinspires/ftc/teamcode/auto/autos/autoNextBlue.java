package org.firstinspires.ftc.teamcode.auto.autos;

import static dev.nextftc.extensions.pedro.PedroComponent.follower;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;

import org.firstinspires.ftc.teamcode.auto.AutoCommands;
import org.firstinspires.ftc.teamcode.auto.pedro.constants.Constants;
import org.firstinspires.ftc.teamcode.auto.subsystems.NextInBetween;
import org.firstinspires.ftc.teamcode.auto.subsystems.NextShooter;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.delays.Delay;
import dev.nextftc.core.commands.groups.SequentialGroup;
import dev.nextftc.core.components.SubsystemComponent;
import dev.nextftc.extensions.pedro.FollowPath;
import dev.nextftc.extensions.pedro.PedroComponent;
import dev.nextftc.ftc.NextFTCOpMode;
import dev.nextftc.ftc.components.BulkReadComponent;

@Autonomous(name = "NextFTC blue",group = "tests")
public class autoNextBlue extends NextFTCOpMode{

    public autoNextBlue() {
        addComponents(
                new SubsystemComponent(NextShooter.INSTANCE, NextInBetween.INSTANCE),
                BulkReadComponent.INSTANCE,
                new PedroComponent(Constants::createFollower)
        );

    }

    private final Pose startPose = new Pose(19, 121.5, Math.toRadians(144)); // Start Pose of our robot.
    private final Pose scorePose = new Pose(47, 95, Math.toRadians(135)); // Scoring Pose of our robot. It is facing the goal at a 135 degree angle.
    private final Pose controlPose = new Pose(70,60);// pose for getting to GPP without hitting other balls
    private final Pose endPose = new Pose(47, 133, Math.toRadians(180));


    private PathChain leavePath, scorePath;
    private Follower follower;
    AutoCommands command = new AutoCommands(follower);

    public void buildPaths() {
        /* This is our scorePreload path. We are using a BezierLine, which is a straight line. */
        scorePath = follower().pathBuilder()
                .addPath(new BezierLine(startPose,scorePose))
                .setLinearHeadingInterpolation(startPose.getHeading(),scorePose.getHeading())
                .build();

        leavePath = follower().pathBuilder()
                .addPath(new BezierLine(scorePose, endPose))
                .setLinearHeadingInterpolation(scorePose.getHeading(), endPose.getHeading())
                .build();

//        scorePreload = new Path(new BezierLine(startPose, scorePose));
//        scorePreload.setLinearHeadingInterpolation(startPose.getHeading(), scorePose.getHeading());
    }

    private Command autonomousRoutine() {
        return new SequentialGroup(
                new FollowPath(scorePath),
                command.shoot(),
                new Delay(5),
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
