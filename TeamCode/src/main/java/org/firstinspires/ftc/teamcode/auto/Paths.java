package org.firstinspires.ftc.teamcode.auto;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;

public class Paths {
    Follower follower;
    public Paths(Follower follower){
        this.follower = follower;
    }
    public final Pose startPoseB = new Pose(19, 121.5, Math.toRadians(144)); // Start Pose of our robot.
    public final Pose scorePoseB = new Pose(47.60172591970307, 95.1073798180677, Math.toRadians(135)); // Scoring Pose of our robot. It is facing the goal at a 135 degree angle.
    public final Pose controlPoseB = new Pose(70,60);// pose for getting to GPP without hitting other balls
    public final Pose GPPB = new Pose(40, 37, Math.toRadians(180));
    public final Pose PPGB = new Pose(42, 84.3, Math.toRadians(180));
    public final Pose PGPB = new Pose(40, 60.5, Math.toRadians(180));

    public final Pose afterPickup1B = new Pose(12, 35, Math.toRadians(180));

    public final Pose afterPickup2B = new Pose(17.5, 84.3, Math.toRadians(180));
    public final Pose afterPickup3B = new Pose(13, 59, Math.toRadians(180));
    public final Pose autoEndPoseB = new Pose(13,59,Math.toRadians(180));
    public void buildPaths() {
        /* This is our scorePreload path. We are using a BezierLine, which is a straight line. */
        scorePreload = follower.pathBuilder()
                .addPath(new BezierLine(startPoseB, scorePoseB))
                .setLinearHeadingInterpolation(startPoseB.getHeading(), scorePoseB.getHeading())
                .build();


    /* Here is an example for Constant Interpolation
    scorePreload.setConstantInterpolation(startPose.getHeading()); */

        /* This is our grabPickup1 PathChain. We are using a single path with a BezierLine, which is a straight line. */
        grabGPP = follower.pathBuilder()
                .addPath(new BezierCurve(scorePoseB, controlPoseB, GPPB))
                .setLinearHeadingInterpolation(scorePoseB.getHeading(), GPPB.getHeading())
                .build();

        intake1 = follower.pathBuilder()
                .addPath(new BezierLine(GPPB, afterPickup1B))
                .build();


        scoreGPP = follower.pathBuilder()
                .addPath(new BezierCurve(afterPickup1B, controlPoseB, scorePoseB))
                .setLinearHeadingInterpolation(afterPickup1B.getHeading(), scorePoseB.getHeading())
                .build();

        grabPPG = follower.pathBuilder()
                .addPath(new BezierLine(scorePoseB, PPGB))
                .setLinearHeadingInterpolation(scorePoseB.getHeading(), PPGB.getHeading())
                .build();

        intake2 = follower.pathBuilder()
                .addPath(new BezierLine(PPGB, afterPickup2B))
                .build();

        scorePPG = follower.pathBuilder()
                .addPath(new BezierLine(afterPickup2B, scorePoseB))
                .setLinearHeadingInterpolation(afterPickup2B.getHeading(), scorePoseB.getHeading())
                .build();

        grabPGP = follower.pathBuilder()
                .addPath(new BezierLine(scorePoseB, PGPB))
                .setLinearHeadingInterpolation(scorePoseB.getHeading(), PGPB.getHeading())
                .build();

        scorePGP = follower.pathBuilder()
                .addPath(new BezierLine(PGPB, scorePoseB))
                .setLinearHeadingInterpolation(PGPB.getHeading(),scorePoseB.getHeading())
                .build();

        intake3 = follower.pathBuilder()
                .addPath(new BezierLine(PGPB, afterPickup3B))
                .build();

        autoEnd = follower.pathBuilder()
                .addPath(new BezierLine(afterPickup3B, autoEndPoseB))
                .setLinearHeadingInterpolation(afterPickup2B.getHeading(), scorePoseB.getHeading())
                .build();

    }
    public PathChain scorePreload, grabGPP, scoreGPP,intake1, intake2, intake3, grabPPG, scorePPG, scorePGP, grabPGP, autoEnd;
}
