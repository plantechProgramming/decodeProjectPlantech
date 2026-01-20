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
    public final Pose startPose = new Pose(20.1, 122.5, Math.toRadians(144)); // Start Pose of our robot.
    public final Pose scorePose = new Pose(47.5, 96, Math.toRadians(135)); // Scoring Pose of our robot. It is facing the goal at a 135 degree angle.
    public final Pose controlPosePPG = new Pose(70,60);// pose for getting to PPG without hitting other balls
    public final Pose controlPosePGP = new Pose(40,55);// pose for getting to PGP without hitting other balls
    public final Pose leaveClosePose = new Pose(60, 122.5, Math.toRadians(180));
    public final Pose GPP = new Pose(40, 37, Math.toRadians(180));
    public final Pose PPG = new Pose(42, 84.3, Math.toRadians(180));
    public final Pose PGP = new Pose(42, 60, Math.toRadians(180));

    public final Pose afterPickupGPP = new Pose(10, 36, Math.toRadians(180));

    public final Pose afterPickupPPG = new Pose(17.5, 84.3, Math.toRadians(180));
    public final Pose afterPickupPGP = new Pose(10, 60, Math.toRadians(180));

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
    public PathChain leaveClose;
    public void buildPaths() {
        scorePreload = follower.pathBuilder()
                .addPath(new BezierLine(startPose, scorePose))
                .setLinearHeadingInterpolation(startPose.getHeading(), scorePose.getHeading())
                .build();

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

        leaveClose = follower.pathBuilder()
                .addPath(new BezierLine(scorePose, leaveClosePose))
                .setLinearHeadingInterpolation(scorePose.getHeading(), leaveClosePose.getHeading())
                .build();

    }

    // these work ONLY FOR BLUE
    PathChain scorePath(Pose startPose){
        return follower.pathBuilder()
                .addPath(new BezierLine(startPose, scorePose))
                .setLinearHeadingInterpolation(startPose.getHeading(), scorePose.getHeading())
                .build();
    }

    PathChain intakePath(Pose startPose, double distance){
        Pose endPose = new Pose(startPose.getX() - distance,startPose.getY(), startPose.getHeading());
        return follower.pathBuilder()
                .addPath(new BezierLine(startPose,endPose))
                .build();
    }


}
