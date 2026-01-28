package org.firstinspires.ftc.teamcode.auto;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.CoordinateSystem;
import com.pedropathing.geometry.PedroCoordinates;
import com.pedropathing.geometry.Pose;
import com.pedropathing.math.MathFunctions;
import com.pedropathing.paths.PathChain;

public class PathsRed {


    public PathsRed(){
    }
    private final PathsBlue pathsBlue = new PathsBlue();
    private final Pose startPose = pathsBlue.getSPose().mirror(); // Start Pose of our robot.

    // the y is way too large on purpose for localization fix
    private final Pose startPoseFar = new Pose(88.3,15.8,Math.toRadians(90)); // Start Pose of our robot.
    private final Pose scorePoseFar = pathsBlue.scorePoseFar.mirror(); // Scoring Pose of our robot. It is facing the goal at a 115 degree angle.

    public final Pose scorePose = pathsBlue.scorePose.mirror(); // Scoring Pose of our robot. It is facing the goal at a 135 degree angle.
    public final Pose controlPosePPG = pathsBlue.controlPosePPG.mirror();// pose for getting to PPG without hitting other balls
    public final Pose controlPosePGP = pathsBlue.controlPosePGP.mirror();// pose for getting to PGP without hitting other balls
    public final Pose controlPoseGPP = pathsBlue.controlPoseGPP.mirror();// pose for getting to PGP without hitting other balls


    public final Pose leaveClosePose = pathsBlue.leaveClosePose.mirror();
//    public final Pose GPP = Mymirror(pathsBlue.GPP);
//    public final Pose PPG = Mymirror(pathsBlue.PPG);
//    public final Pose PGP = Mymirror(pathsBlue.PGP);
//
//    public final Pose afterPickupGPP = Mymirror(pathsBlue.afterPickupGPP);
//    public final Pose afterPickupPPG = Mymirror(pathsBlue.afterPickupPPG);
//    public final Pose afterPickupPGP = Mymirror(pathsBlue.afterPickupPGP);

    public final Pose GPP = pathsBlue.GPP.mirror();
    public final Pose PPG = pathsBlue.PPG.mirror();
    public final Pose PGP = pathsBlue.PGP.mirror();

    public final Pose afterPickupGPP = pathsBlue.afterPickupGPP.mirror();
    public final Pose afterPickupPPG = pathsBlue.afterPickupPPG.mirror();
    public final Pose afterPickupPGP = pathsBlue.afterPickupPGP.mirror();

    public final Pose leaveFarPose = pathsBlue.leaveFarPose.mirror();

    public PathChain scorePreload, scorePreloadFar;
    public PathChain grabGPP, grabPGP, grabPPG;
    public PathChain scoreGPP, scorePGP, scorePPG;
    public PathChain intakeGPP, intakePGP, intakePPG;
    public PathChain grabGPPFar, grabPGPFar;
    public PathChain scoreGPPFar;

    public PathChain scoreLeaveClose, scoreLeaveFar;
    public PathChain leaveClose, leaveFar;

    private Pose Mymirror(Pose pPose) {
        Pose k = pPose;
        return new Pose(144 - k.getX(), k.getY()-17/2.54, MathFunctions.normalizeAngle(Math.PI - k.getHeading()), PedroCoordinates.INSTANCE);
    }

    public Pose getSPoseFar(){
        return this.startPoseFar;
    }
    public Pose getSPose(){
        return this.startPose;
    }

    public void buildPaths(Follower follower) {
        scorePreload = follower.pathBuilder()
                .addPath(new BezierLine(startPose, scorePose))
                .setLinearHeadingInterpolation(startPose.getHeading(), scorePose.getHeading())
                .build();
        scorePreloadFar =  follower.pathBuilder()
                .addPath(new BezierLine(startPoseFar, scorePoseFar))
                .setLinearHeadingInterpolation(startPoseFar.getHeading(), scorePoseFar.getHeading())
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

        scoreLeaveClose = follower.pathBuilder()
                .addPath(new BezierLine(scorePose, leaveClosePose))
                .setLinearHeadingInterpolation(scorePose.getHeading(), leaveClosePose.getHeading())
                .build();

        leaveClose = follower.pathBuilder()
                .addPath(new BezierLine(startPose, leaveClosePose))
                .setLinearHeadingInterpolation(startPose.getHeading(), leaveClosePose.getHeading())
                .build();

        scoreLeaveFar = follower.pathBuilder()
                .addPath(new BezierLine(scorePose, leaveFarPose))
                .setLinearHeadingInterpolation(scorePose.getHeading(), leaveFarPose.getHeading())
                .build();

        leaveFar = follower.pathBuilder()
                .addPath(new BezierLine(startPose, leaveFarPose))
                .setLinearHeadingInterpolation(startPose.getHeading(), leaveFarPose.getHeading())
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