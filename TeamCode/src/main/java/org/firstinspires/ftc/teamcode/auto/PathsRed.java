package org.firstinspires.ftc.teamcode.auto;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;

public class PathsRed {

    private Follower follower;
    public PathsRed(Follower follower){
        this.follower = follower;
    }
    private final PathsBlue pathsBlue = new PathsBlue(follower);

    public final Pose startPose = pathsBlue.startPose.mirror(); // Start Pose of our robot.
    public final Pose startPoseFar = pathsBlue.startPoseFar.mirror(); // Start Pose of our robot.
    private final Pose scorePoseFar = pathsBlue.scorePoseFar.mirror(); // Scoring Pose of our robot. It is facing the goal at a 115 degree angle.

    public final Pose scorePose = pathsBlue.scorePose.mirror(); // Scoring Pose of our robot. It is facing the goal at a 135 degree angle.
    public final Pose controlPosePPG = pathsBlue.controlPosePPG.mirror();// pose for getting to PPG without hitting other balls
    public final Pose controlPosePGP = pathsBlue.controlPosePGP.mirror();// pose for getting to PGP without hitting other balls
    public final Pose controlPoseGPP = pathsBlue.controlPoseGPP.mirror();// pose for getting to PGP without hitting other balls


    public final Pose leaveClosePose = pathsBlue.leaveClosePose.mirror();
    public final Pose GPP = pathsBlue.GPP.mirror();
    public final Pose PPG = pathsBlue.PPG.mirror();
    public final Pose PGP = pathsBlue.PGP.mirror();

    public final Pose afterPickupGPP = pathsBlue.afterPickupGPP.mirror();

    public final Pose afterPickupPPG = pathsBlue.afterPickupPPG.mirror();
    public final Pose afterPickupPGP = pathsBlue.afterPickupPGP.mirror();

    public PathChain scorePreload;
    public PathChain scorePreloadFar;
    public PathChain grabGPP, grabPGP, grabPPG;
    public PathChain scoreGPP, scorePGP, scorePPG;
    public PathChain intakeGPP, intakePGP, intakePPG;
    public PathChain grabGPPFar, grabPGPFar;
    public PathChain scoreGPPFar;

    public PathChain leaveClose;


    public void buildPaths() {
        scorePreload = follower.pathBuilder()
                .addPath(new BezierLine(startPose, scorePose))
                .setLinearHeadingInterpolation(startPose.getHeading(), scorePose.getHeading())
                .build();
        scorePreloadFar =  follower.pathBuilder()
                .addPath(new BezierLine(startPoseFar, scorePoseFar))
                .setLinearHeadingInterpolation(startPose.getHeading(), scorePoseFar.getHeading())
                .build();

        grabGPP = follower.pathBuilder()
                .addPath(new BezierCurve(scorePose, controlPosePPG, GPP))
                .setLinearHeadingInterpolation(scorePose.getHeading(), GPP.getHeading())
                .build();

        intakeGPP = follower.pathBuilder()
                .addPath(new BezierLine(GPP, afterPickupGPP))
                .build();


        scoreGPP = follower.pathBuilder()
                .addPath(new BezierCurve(afterPickupGPP, controlPoseGPP, scorePose))
                .setLinearHeadingInterpolation(afterPickupGPP.getHeading(), scorePose.getHeading())
                .build();

        grabPPG = follower.pathBuilder()
                .addPath(new BezierLine(scorePose, PPG))
                .setLinearHeadingInterpolation(scorePose.getHeading(), PPG.getHeading())
                .build();

//        grabPPGFar = follower.pathBuilder()
//                .addPath(new BezierLine(scorePose, PPG))
//                .setLinearHeadingInterpolation(scorePose.getHeading(), PPG.getHeading())
//                .build();

        intakePPG = follower.pathBuilder()
                .addPath(new BezierLine(PPG, afterPickupPPG))
                .build();

        scorePPG = follower.pathBuilder()
                .addPath(new BezierLine(afterPickupPPG, scorePose))
                .setLinearHeadingInterpolation(afterPickupPPG.getHeading(), scorePose.getHeading())
                .build();

        grabPGP = follower.pathBuilder()
                .addPath(new BezierLine(scorePose, PGP))
                .setLinearHeadingInterpolation(scorePose.getHeading(), PGP.getHeading())
                .build();

        grabPGPFar = follower.pathBuilder()
                .addPath(new BezierLine(scorePoseFar, PGP))
                .setLinearHeadingInterpolation(scorePose.getHeading(), PGP.getHeading())
                .build();

        scorePGP = follower.pathBuilder()
                .addPath(new BezierCurve(afterPickupPGP, controlPosePGP, scorePose))
                .setLinearHeadingInterpolation(PGP.getHeading(),scorePose.getHeading())
                .build();

        intakePGP = follower.pathBuilder()
                .addPath(new BezierLine(PGP, afterPickupPGP))
                .build();

        leaveClose = follower.pathBuilder()
                .addPath(new BezierLine(scorePose, leaveClosePose))
                .setLinearHeadingInterpolation(scorePose.getHeading(), leaveClosePose.getHeading())
                .build();
        grabGPPFar = follower.pathBuilder()
                .addPath(new BezierLine(scorePoseFar, GPP))
                .setLinearHeadingInterpolation(scorePoseFar.getHeading(), GPP.getHeading())
                .build();

        scoreGPPFar = follower.pathBuilder()
                .addPath(new BezierLine(afterPickupGPP, scorePoseFar))
                .setLinearHeadingInterpolation(afterPickupGPP.getHeading(), scorePoseFar.getHeading())
                .build();

    }
}
