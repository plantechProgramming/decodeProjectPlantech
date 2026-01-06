package org.firstinspires.ftc.teamcode.auto;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;

public class PathsBlue {
    Follower follower;
    public PathsBlue(Follower follower){
        this.follower = follower;
    }
    public final Pose startPose = new Pose(19, 121.5, Math.toRadians(144)); // Start Pose of our robot.
    public final Pose scorePose = new Pose(47.60172591970307, 95.1073798180677, Math.toRadians(135)); // Scoring Pose of our robot. It is facing the goal at a 135 degree angle.
    public final Pose controlPosePPG = new Pose(70,60);// pose for getting to PPG without hitting other balls
    public final Pose controlPosePGP = new Pose(40,55);// pose for getting to PGP without hitting other balls
    public final Pose GPP = new Pose(40, 37, Math.toRadians(180));
    public final Pose PPG = new Pose(42, 84.3, Math.toRadians(180));
    public final Pose PGP = new Pose(41.5, 60, Math.toRadians(180));

    public final Pose afterPickupGPP = new Pose(12.5, 35, Math.toRadians(180));

    public final Pose afterPickupPPG = new Pose(17.5, 84.3, Math.toRadians(180));
    public final Pose afterPickupPGP = new Pose(13, 59, Math.toRadians(180));
    public final Pose autoEndPose = new Pose(13,59,Math.toRadians(180));

    public PathChain scorePreload;
    public PathChain grabGPP;
    public PathChain scoreGPP;
    public PathChain intakeGPP;
    public PathChain intakePPG;
    public PathChain intakePGP;
    public PathChain grabPPG;
    public PathChain scorePPG;
    public PathChain scorePGP;
    public PathChain grabPGP;
    public PathChain autoEnd;
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
                .addPath(new BezierCurve(scorePose, controlPosePPG, GPP))
                .setLinearHeadingInterpolation(scorePose.getHeading(), GPP.getHeading())
                .build();

        intakeGPP = follower.pathBuilder()
                .addPath(new BezierLine(GPP, afterPickupGPP))
                .build();


        scoreGPP = follower.pathBuilder()
                .addPath(new BezierCurve(afterPickupGPP, controlPosePPG, scorePose))
                .setLinearHeadingInterpolation(afterPickupGPP.getHeading(), scorePose.getHeading())
                .build();

        grabPPG = follower.pathBuilder()
                .addPath(new BezierLine(scorePose, PPG))
                .setLinearHeadingInterpolation(scorePose.getHeading(), PPG.getHeading())
                .build();

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

        scorePGP = follower.pathBuilder()
                .addPath(new BezierCurve(afterPickupPGP, controlPosePGP, scorePose))
                .setLinearHeadingInterpolation(PGP.getHeading(),scorePose.getHeading())
                .build();

        intakePGP = follower.pathBuilder()
                .addPath(new BezierLine(PGP, afterPickupPGP))
                .build();

        autoEnd = follower.pathBuilder()
                .addPath(new BezierLine(afterPickupPGP, autoEndPose))
                .setLinearHeadingInterpolation(afterPickupPPG.getHeading(), scorePose.getHeading())
                .build();

    }
}
