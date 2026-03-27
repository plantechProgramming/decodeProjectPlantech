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
//    private final Pose startPoseFar = new Pose(85.35,15,Math.toRadians(90)); // Start Pose of our robot.
    private final Pose startPoseFar = pathsBlue.getSPoseFar().mirror();
    private final Pose scorePoseFar = pathsBlue.scorePoseFar.mirror(); // Scoring Pose of our robot. It is facing the goal at a 115 degree angle.
    public final Pose scorePose = pathsBlue.scorePose.mirror(); // Scoring Pose of our robot. It is facing the goal at a 135 degree angle.
    public final Pose leaveClosePose = pathsBlue.leaveClosePose.mirror();
////
//
//    public final Pose startPose = new Pose(124.000, 121.700, Math.toRadians(36));
//    public final Pose scorePoseFar = new Pose(88, 16, Math.toRadians(70));
////    public final Pose scorePose = new Pose(94.981, 97.019, Math.toRadians(39));
//    public final Pose startPoseFar = new Pose(88, 8.5, Math.toRadians(90));
//    public final Pose GPP = new Pose(101.8, 35, Math.toRadians(0));
//    public final Pose PPG =  new Pose(103.000, 84.00, Math.toRadians(0));
//    public final Pose PGP = new Pose(103.000, 60.000, Math.toRadians(0));
//
//    public final Pose afterPickupGPP = new Pose(132.700, 36.000, Math.toRadians(0));
//    public final Pose afterPickupPPG = new Pose(127.600, 83.500, Math.toRadians(0));
//    public final Pose afterPickupPGP = new Pose(133.700, 58.500, Math.toRadians(0));


    public final Pose controlPosePPG = pathsBlue.controlPosePPG.mirror();// pose for getting to PPG without hitting other balls
    public final Pose controlPosePGP = pathsBlue.controlPosePGP.mirror();// pose for getting to PGP without hitting other balls
    public final Pose controlPoseGPP = pathsBlue.controlPoseGPP.mirror();// pose for getting to PGP without hitting other balls
    public final Pose controlPoseGate = pathsBlue.controlPoseGate.mirror();
    public final Pose controlPoseGatePPG = pathsBlue.controlPoseGatePPG.mirror();
    public final Pose gate = pathsBlue.gate.mirror();
    public final Pose GPP = pathsBlue.GPP.mirror();
    public final Pose PPG = pathsBlue.PPG.mirror();
    public final Pose PGP = pathsBlue.PGP.mirror();

    public final Pose afterPickupGPP = pathsBlue.afterPickupGPP.mirror();
    public final Pose afterPickupPPG = pathsBlue.afterPickupPPG.mirror();
    public final Pose afterPickupPGP = pathsBlue.afterPickupPGP.mirror();
//
    public final Pose leaveFarPose = pathsBlue.leaveFarPose.mirror();

    public PathChain scorePreload, scorePreloadFar;
    public PathChain grabGPP, grabPGP, grabPPG;
    public PathChain scoreGPP, scorePGP, scorePPG;
    public PathChain grabGPPFar, grabPGPFar;
    public PathChain scoreGPPFar;
    public PathChain scoreGateFromPGP;
    public PathChain scoreLeaveClose, leavePPGClose, leaveClose;
    public PathChain scoreLeaveFar, leaveFar;

    private Pose Mymirror(Pose pPose) {
        Pose k = pPose;
        return new Pose(141.5 - k.getX(), k.getY()-2, MathFunctions.normalizeAngle(Math.PI - k.getHeading()), PedroCoordinates.INSTANCE);
    }

    public Pose getSPoseFar(){
        return this.startPoseFar;
    }
    public Pose getSPose(){
        return this.startPose;
    }

    public void buildPaths(Follower follower) {

        //-------------PRELOAD------------------

        scorePreload = follower.pathBuilder()
                .addPath(new BezierLine(startPose, scorePose))
                .setLinearHeadingInterpolation(startPose.getHeading(), scorePose.getHeading())
                .build();
        scorePreloadFar =  follower.pathBuilder()
                .addPath(new BezierLine(startPoseFar, scorePoseFar))
                .setLinearHeadingInterpolation(startPoseFar.getHeading(), scorePoseFar.getHeading())
                .build();

        //-------------GRAB && SCORE --CLOSE------------------

        grabGPP = follower.pathBuilder()
                .addPath(new BezierCurve(scorePose, controlPosePPG, GPP))
                .setLinearHeadingInterpolation(scorePose.getHeading(), GPP.getHeading())
                .addPath(new BezierLine(GPP, afterPickupGPP))
                .build();


        scoreGPP = follower.pathBuilder()
                .addPath(new BezierCurve(afterPickupGPP, controlPoseGPP, scorePose))
                .setLinearHeadingInterpolation(afterPickupGPP.getHeading(), scorePose.getHeading())
                .build();

        grabPPG = follower.pathBuilder()
                .addPath(new BezierLine(scorePose, PPG))
                .setLinearHeadingInterpolation(scorePose.getHeading(), PPG.getHeading())
                .addPath(new BezierLine(PPG, afterPickupPPG))
                .build();

        scorePPG = follower.pathBuilder()
                .addPath(new BezierLine(afterPickupPPG, scorePose))
                .setLinearHeadingInterpolation(afterPickupPPG.getHeading(), scorePose.getHeading())
                .build();

        grabPGP = follower.pathBuilder()
                .addPath(new BezierCurve(scorePose, controlPosePGP, PGP))
                .setLinearHeadingInterpolation(scorePose.getHeading(), PGP.getHeading())
                .addPath(new BezierLine(PGP, afterPickupPGP))
                .build();

        scorePGP = follower.pathBuilder()
                .addPath(new BezierCurve(afterPickupPGP, controlPosePGP, scorePose))
                .setLinearHeadingInterpolation(PGP.getHeading(),scorePose.getHeading())
                .build();

        //-------------GRAB && SCORE --FAR------------------

        grabPGPFar = follower.pathBuilder()
                .addPath(new BezierLine(scorePoseFar, PGP))
                .setLinearHeadingInterpolation(scorePose.getHeading(), PGP.getHeading())
                .addPath(new BezierLine(PGP, afterPickupPGP))
                .build();

        grabGPPFar = follower.pathBuilder()
                .addPath(new BezierLine(scorePoseFar, GPP))
                .setLinearHeadingInterpolation(scorePoseFar.getHeading(), GPP.getHeading())
                .addPath(new BezierLine(GPP, afterPickupGPP))
                .build();

        scoreGPPFar = follower.pathBuilder()
                .addPath(new BezierLine(afterPickupGPP, scorePoseFar))
                .setLinearHeadingInterpolation(afterPickupGPP.getHeading(), scorePoseFar.getHeading())
                .build();

        //-------------LEAVE------------------

        scoreLeaveClose = follower.pathBuilder()
                .addPath(new BezierLine(scorePose, leaveClosePose))
                .setLinearHeadingInterpolation(scorePose.getHeading(), leaveClosePose.getHeading())
                .build();

        leaveClose = follower.pathBuilder()
                .addPath(new BezierLine(startPose, leaveClosePose))
                .setLinearHeadingInterpolation(startPose.getHeading(), leaveClosePose.getHeading())
                .build();

        leavePPGClose = follower.pathBuilder()
                .addPath(new BezierLine(afterPickupPPG, leaveClosePose))
                .setLinearHeadingInterpolation(afterPickupPPG.getHeading(), leaveClosePose.getHeading())
                .build();

        scoreLeaveFar = follower.pathBuilder()
                .addPath(new BezierLine(scorePose, leaveFarPose))
                .setLinearHeadingInterpolation(scorePose.getHeading(), leaveFarPose.getHeading())
                .build();

        leaveFar = follower.pathBuilder()
                .addPath(new BezierLine(startPose, leaveFarPose))
                .setLinearHeadingInterpolation(startPose.getHeading(), leaveFarPose.getHeading())
                .build();

        //-------------Gate------------------

        scoreGateFromPGP = follower.pathBuilder()
                .addPath(new BezierCurve(afterPickupPGP, controlPoseGate, gate))
                .setLinearHeadingInterpolation(afterPickupPGP.getHeading(), gate.getHeading())
                .addPath(new BezierCurve(gate, controlPoseGatePPG, scorePose))
                .setLinearHeadingInterpolation(gate.getHeading(), scorePose.getHeading())
                .build();

//        scoreCloseGateFromScore = follower.pathBuilder()
//                .addPath(new BezierCurve(scorePose, controlPosePGP, PGP))
//                .setLinearHeadingInterpolation(scorePose.getHeading(), PGP.getHeading())
//                .setVelocityConstraint(0.1)
//                .addPath(new BezierLine(PGP, afterPickupPGP))
//                .addPath(new BezierCurve(afterPickupPGP,controlPoseGate,gate))
//                .setLinearHeadingInterpolation(afterPickupPGP.getHeading(), gate.getHeading())
//                .addPath(new BezierCurve(gate, controlPoseGatePPG,scorePose))
//                .setLinearHeadingInterpolation(gate.getHeading(), scorePose.getHeading())
//                .build();
//
//        leaveClosePPGFromScore = follower.pathBuilder()
//                .addPath(new BezierLine(scorePose, PPG))
//                .setLinearHeadingInterpolation(scorePose.getHeading(), PPG.getHeading())
//                .addPath(new BezierLine(GPP, afterPickupGPP))
//                .addPath(new BezierLine(afterPickupPPG, leaveClosePose))
//                .setLinearHeadingInterpolation(afterPickupPPG.getHeading(), leaveClosePose.getHeading())
//                .build();



    }
}