package org.firstinspires.ftc.teamcode.auto.autos;

import static dev.nextftc.extensions.pedro.PedroComponent.follower;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;

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
import dev.nextftc.core.components.SubsystemComponent;
import dev.nextftc.extensions.pedro.FollowPath;
import dev.nextftc.extensions.pedro.PedroComponent;
import dev.nextftc.ftc.NextFTCOpMode;
import dev.nextftc.ftc.components.BulkReadComponent;

@Autonomous(name = "Plan A Next blue")
public class PlanANextBlue extends NextFTCOpMode{

    public PlanANextBlue() {
        addComponents(
                new SubsystemComponent(NextShooter.INSTANCE, NextInBetween.INSTANCE),
                BulkReadComponent.INSTANCE,
                new PedroComponent(Constants::createFollower)
        );

    }



    private final Pose startPose = new Pose(19, 121.5, Math.toRadians(144)); // Start Pose of our robot.
    private final Pose scorePose = new Pose(47.60172591970307, 95.1073798180677, Math.toRadians(135)); // Scoring Pose of our robot. It is facing the goal at a 135 degree angle.
    private final Pose controlPose = new Pose(70,60);// pose for getting to GPP without hitting other balls
    private final Pose GPP = new Pose(40, 37, Math.toRadians(180));
    private final Pose PPG = new Pose(42, 84.3, Math.toRadians(180));
    private final Pose PGP = new Pose(40, 59, Math.toRadians(180));

    private final Pose afterPickup1 = new Pose(11.5, 35, Math.toRadians(180));

    private final Pose afterPickup2 = new Pose(17.5, 84.3, Math.toRadians(180));
    private final Pose afterPickup3 = new Pose(15, 59, Math.toRadians(180));
    private final Pose autoEndPose = new Pose(15,59,Math.toRadians(180));

    private PathChain scorePreload, grabGPP, scoreGPP,intake1, intake2, intake3, grabPPG, scorePPG, grabPGP, autoEnd;
    private Follower follower;
    AutoCommands command = new AutoCommands(follower);

    public void buildPaths() {
        /* This is our scorePreload path. We are using a BezierLine, which is a straight line. */
        scorePreload = follower.pathBuilder()
                .addPath(new BezierLine(startPose, scorePose))
                .setLinearHeadingInterpolation(startPose.getHeading(), scorePose.getHeading())
                .build();


    /* Here is an example for Constant Interpolation
    scorePreload.setConstantInterpolation(startPose.getHeading()); */

        /* This is our grabPickup1 PathChain. We are using a single path with a BezierLine, which is a straight line. */
        grabGPP = follower.pathBuilder()
                .addPath(new BezierCurve(scorePose, controlPose, GPP))
                .setLinearHeadingInterpolation(scorePose.getHeading(), GPP.getHeading())
                .build();

        intake1 = follower.pathBuilder()
                .addPath(new BezierLine(GPP, afterPickup1))
                .build();


        scoreGPP = follower.pathBuilder()
                .addPath(new BezierCurve(afterPickup1, controlPose, scorePose))
                .setLinearHeadingInterpolation(afterPickup1.getHeading(), scorePose.getHeading())
                .build();

        grabPPG = follower.pathBuilder()
                .addPath(new BezierLine(scorePose, PPG))
                .setLinearHeadingInterpolation(scorePose.getHeading(), PPG.getHeading())
                .build();

        intake2 = follower.pathBuilder()
                .addPath(new BezierLine(PPG, afterPickup2))
                .build();

        scorePPG = follower.pathBuilder()
                .addPath(new BezierLine(afterPickup2, scorePose))
                .setLinearHeadingInterpolation(afterPickup2.getHeading(), scorePose.getHeading())
                .build();

        grabPGP = follower.pathBuilder()
                .addPath(new BezierLine(scorePose, PGP))
                .setLinearHeadingInterpolation(scorePose.getHeading(), PGP.getHeading())
                .build();

        intake3 = follower.pathBuilder()
                .addPath(new BezierLine(PGP, afterPickup3))
                .build();

        autoEnd = follower.pathBuilder()
                .addPath(new BezierLine(afterPickup3, autoEndPose))
                .setLinearHeadingInterpolation(afterPickup2.getHeading(), scorePose.getHeading())
                .build();

    }

    public Command autoRoutine(){
        return new SequentialGroup(
                command.startShooter(false),
                new Delay(2),
                command.score(scorePreload),
                new Delay(4),
                command.intake(intake1,grabGPP,0.55),
                command.score(scoreGPP),
                new Delay(4),
                command.intake(intake2,grabPPG,0.55),
                command.score(scorePPG)
        );
    }
    @Override
    public void onStartButtonPressed() {
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startPose);
        buildPaths();
        autoRoutine().schedule();
    }

}
