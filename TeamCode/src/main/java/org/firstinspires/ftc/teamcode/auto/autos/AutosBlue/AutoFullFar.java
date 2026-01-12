package org.firstinspires.ftc.teamcode.auto.autos.AutosBlue; // make sure this aligns with class location
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
import org.firstinspires.ftc.teamcode.auto.pedro.constants.Constants;

import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.groups.SequentialGroup;
import dev.nextftc.ftc.NextFTCOpMode;

@Autonomous(name = "AutoFullFar")
public class AutoFullFar extends NextFTCOpMode {
    private Follower follower;
//    public void AutoFullFar() {
//        addComponents(
//                new SubsystemComponent(NextShooter.INSTANCE, NextInBetween.INSTANCE),
//                BulkReadComponent.INSTANCE,
//                new PedroComponent(Constants::createFollower)
//        );
//
//    }
    private Timer pathTimer, actionTimer, opmodeTimer;
    private int pathState;
    AutoCommands command = new AutoCommands(follower());
    PathsBlue path = new PathsBlue(follower());


    public final Pose startPoseFar = new Pose(62, 8, Math.toRadians(180)); // Start Pose of our robot for the far position.
    private final Pose scorePoseFar = new Pose(62, 16, Math.toRadians(135)); // Scoring Pose of our robot. It is facing the goal at a 135 degree angle.
    private final Pose scorePose = new Pose(47.60172591970307, 95.1073798180677, Math.toRadians(135)); // Scoring Pose of our robot. It is facing the goal at a 135 degree angle.
    private final Pose controlPose = new Pose(50,60);// pose for getting to GPP without hitting other balls
    private final Pose GPP = new Pose(40, 35, Math.toRadians(180));
    private final Pose PPG = new Pose(40, 85, Math.toRadians(180));
    private final Pose PGP = new Pose(40, 60, Math.toRadians(180));

    private final Pose afterPickup1 = new Pose(15, 35, Math.toRadians(180));

    private final Pose afterPickup2 = new Pose(15, 84.3, Math.toRadians(180));
    private final Pose afterPickup3 = new Pose(15, 59, Math.toRadians(180));
    private final Pose autoEndPose = new Pose(15,59,Math.toRadians(180));


    // Highest (First Set) of Artifacts from the Spike Mark.
//    private final Pose pickup2Pose = new Pose(43, 130, Math.toRadians(0)); // Middle (Second Set) of Artifacts from the Spike Mark.
//    private final Pose pickup3Pose = new Pose(49, 135, Math.toRadians(0)); // Lowest (Third Set) of Artifacts from the Spike Mark.
    private Path scorePreload;
    private PathChain grabGPP, intake1, scoreGPP,intake2, intake3, grabPPG, scorePGP, grabPGP, autoEnd;

    public void buildPaths() {
        /* This is our scorePreloadClose path. We are using a BezierLine, which is a straight line. */
        scorePreload = new Path(new BezierLine(startPoseFar, scorePoseFar));
        scorePreload.setLinearHeadingInterpolation(startPoseFar.getHeading(), scorePoseFar.getHeading());

    /* Here is an example for Constant Interpolation
    scorePreloadClose.setConstantInterpolation(startPose.getHeading()); */

        /* This is our grabPickup1 PathChain. We are using a single path with a BezierLine, which is a straight line. */
        grabGPP = follower.pathBuilder()
                .addPath(new BezierCurve(scorePose,controlPose, GPP))
                .setLinearHeadingInterpolation(scorePose.getHeading(), GPP.getHeading())
                .build();

        intake1 = follower.pathBuilder()
                .addPath(new BezierLine(GPP, afterPickup1))
                .build();
        scoreGPP = follower.pathBuilder()
                .addPath(new BezierCurve(afterPickup1,controlPose, scorePoseFar))
                .setLinearHeadingInterpolation(afterPickup1.getHeading(), scorePoseFar.getHeading())
                .build();

        grabPPG = follower.pathBuilder()
                .addPath(new BezierLine(scorePose, PPG))
                .setLinearHeadingInterpolation(scorePose.getHeading(), PPG.getHeading())
                .build();

        intake2 = follower.pathBuilder()
                .addPath(new BezierLine(PPG, afterPickup2))
                .build();

        scorePGP = follower.pathBuilder()
                .addPath(new BezierLine(afterPickup2, scorePose))
                .setLinearHeadingInterpolation(afterPickup2.getHeading(), scorePose.getHeading())
                .build();
        grabPGP = follower.pathBuilder()
                .addPath(new BezierLine(scorePoseFar, PGP))
                .setLinearHeadingInterpolation(scorePoseFar.getHeading(), PGP.getHeading())
                .build();

        intake3 = follower.pathBuilder()
                .addPath(new BezierLine(PGP, afterPickup3))
                .build();

        autoEnd = follower.pathBuilder()
                .addPath(new BezierLine(afterPickup3, autoEndPose))
                .setLinearHeadingInterpolation(afterPickup2.getHeading(), scorePose.getHeading())
                .build();
    }

//    public Command preload_1(){
//        return new SequentialGroup(
//        new FollowPath(scorePreloadClose),
////        NextShooter.INSTANCE.naiveShooter(false).invoke()
//        );
//    }
public Command autoRoutine(){
    path.buildPaths();
    return new SequentialGroup(
            command.startShooter(false),
            command.score(path.scorePreloadFar),
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
        follower.setStartingPose(startPoseFar);
        command = new AutoCommands(follower);
        path = new PathsBlue(follower());
        path.buildPaths();
        autoRoutine().schedule();

    }

}