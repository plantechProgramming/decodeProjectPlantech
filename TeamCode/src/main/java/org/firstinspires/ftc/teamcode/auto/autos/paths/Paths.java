package org.firstinspires.ftc.teamcode.auto.autos.paths;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.paths.PathChain;

public class Paths {

    public PathChain scorePreload, scorePreloadFar;
    public PathChain grabGPP, grabPGP, grabPPG;
    public PathChain scoreGPP, scorePGP, scorePPG;
    public PathChain grabGPPFar, grabPGPFar;
    public PathChain scoreGPPFar;
    public PathChain scoreGateFromPGP, pickUpOpenGateFromScore, scorePickUpGate;
    public PathChain scoreHumanPlayer, grabHumanPlayer, grabHumanPlayerTangent;
    public PathChain grabLeftoverBallsGate, scoreLeftoverBallsGate;
    public PathChain scoreLeaveClose, leavePPGClose, leaveClose;
    public PathChain scoreLeaveFar, leaveFar;
    public Points points;

    public Paths(String team){
         points = new Points();
        if(team.equals("RED")) {
            points.setPointsToRed();
        }
    }

    public void buildPaths(Follower follower) {
        //-------------PRELOAD------------------

        scorePreload = follower.pathBuilder()
                .addPath(new BezierLine(Points.startPose, points.scorePose))
                .setLinearHeadingInterpolation(Points.startPose.getHeading(), points.scorePose.getHeading())
                .build();
        scorePreloadFar =  follower.pathBuilder()
                .addPath(new BezierLine(Points.startPoseFar, points.scorePoseFar))
                .setLinearHeadingInterpolation(Points.startPoseFar.getHeading(), points.scorePoseFar.getHeading())
                .build();

        //-------------GRAB && SCORE --CLOSE------------------

        grabGPP = follower.pathBuilder()
                .addPath(new BezierCurve(points.scorePose, points.controlPosePPG, points.GPP))
                .setLinearHeadingInterpolation(points.scorePose.getHeading(), points.GPP.getHeading())
                .addPath(new BezierLine(points.GPP, points.afterPickupGPP))
                .build();


        scoreGPP = follower.pathBuilder()
                .addPath(new BezierCurve(points.afterPickupGPP, points.controlPoseGPP, points.scorePose))
                .setLinearHeadingInterpolation(points.afterPickupGPP.getHeading(), points.scorePose.getHeading())
                .build();

        grabPPG = follower.pathBuilder()
                .addPath(new BezierLine(points.scorePose, points.PPG))
                .setLinearHeadingInterpolation(points.scorePose.getHeading(), points.PPG.getHeading())
                .addPath(new BezierLine(points.PPG, points.afterPickupPPG))
                .build();

        scorePPG = follower.pathBuilder()
                .addPath(new BezierLine(points.afterPickupPPG, points.scorePose))
                .setLinearHeadingInterpolation(points.afterPickupPPG.getHeading(), points.scorePose.getHeading())
                .build();

        grabPGP = follower.pathBuilder()
                .addPath(new BezierCurve(points.scorePose, points.controlPosePGP, points.PGP))
                .setLinearHeadingInterpolation(points.scorePose.getHeading(), points.PGP.getHeading())
                .addPath(new BezierLine(points.PGP, points.afterPickupPGP))
                .build();

        scorePGP = follower.pathBuilder()
                .addPath(new BezierCurve(points.afterPickupPGP, points.controlPosePGP, points.scorePose))
                .setLinearHeadingInterpolation(points.PGP.getHeading(),points.scorePose.getHeading())
                .build();

        //-------------GRAB && SCORE --FAR------------------

        grabPGPFar = follower.pathBuilder()
                .addPath(new BezierLine(points.scorePoseFar, points.PGP))
                .setLinearHeadingInterpolation(points.scorePose.getHeading(), points.PGP.getHeading())
                .addPath(new BezierLine(points.PGP, points.afterPickupPGP))
                .build();

        grabGPPFar = follower.pathBuilder()
                .addPath(new BezierLine(points.scorePoseFar, points.GPP))
                .setLinearHeadingInterpolation(points.scorePoseFar.getHeading(), points.GPP.getHeading())
                .addPath(new BezierLine(points.GPP, points.afterPickupGPP))
                .build();

        scoreGPPFar = follower.pathBuilder()
                .addPath(new BezierLine(points.afterPickupGPP, points.scorePoseFar))
                .setLinearHeadingInterpolation(points.afterPickupGPP.getHeading(), points.scorePoseFar.getHeading())
                .build();

        //-------------LEAVE------------------

        scoreLeaveClose = follower.pathBuilder()
                .addPath(new BezierLine(points.scorePose, points.leaveClosePose))
                .setLinearHeadingInterpolation(points.scorePose.getHeading(), points.leaveClosePose.getHeading())
                .build();

        leaveClose = follower.pathBuilder()
                .addPath(new BezierLine(Points.startPose, points.leaveClosePose))
                .setLinearHeadingInterpolation(Points.startPose.getHeading(), points.leaveClosePose.getHeading())
                .build();

        leavePPGClose = follower.pathBuilder()
                .addPath(new BezierLine(points.afterPickupPPG, points.leaveClosePose))
                .setLinearHeadingInterpolation(points.afterPickupPPG.getHeading(), points.leaveClosePose.getHeading())
                .build();

        scoreLeaveFar = follower.pathBuilder()
                .addPath(new BezierLine(points.scorePose, points.leaveFarPose))
                .setLinearHeadingInterpolation(points.scorePose.getHeading(), points.leaveFarPose.getHeading())
                .build();

        leaveFar = follower.pathBuilder()
                .addPath(new BezierLine(Points.startPose, points.leaveFarPose))
                .setLinearHeadingInterpolation(Points.startPose.getHeading(), points.leaveFarPose.getHeading())
                .build();

        //-------------Gate------------------

        scoreGateFromPGP = follower.pathBuilder()
                .addPath(new BezierCurve(points.afterPickupPGP, points.controlPoseGate, points.gate))
                .setLinearHeadingInterpolation(points.afterPickupPGP.getHeading(), points.gate.getHeading())
                .addPath(new BezierCurve(points.gate, points.controlPoseGatePPG, points.scorePose))
                .setLinearHeadingInterpolation(points.gate.getHeading(), points.scorePose.getHeading())
                .build();

        pickUpOpenGateFromScore = follower.pathBuilder()
                .addPath(new BezierCurve(points.scorePose, points.controlPoseGatePPG, points.gate))
                .setLinearHeadingInterpolation(points.scorePose.getHeading(), points.gate.getHeading())
                .addPath(new BezierLine(points.gate, points.pickUpGate))
                .setLinearHeadingInterpolation(points.gate.getHeading(), points.pickUpGate.getHeading())
                .build();

        scorePickUpGate = follower.pathBuilder()
                .addPath(new BezierCurve(points.pickUpGate, points.controlPoseGatePPG, points.scorePose))
                .setLinearHeadingInterpolation(points.pickUpGate.getHeading(), points.scorePose.getHeading())
                .build();

        // ------------- HUMAN PLAYER ------------------

        grabHumanPlayer = follower.pathBuilder()
                .addPath(new BezierCurve(points.scorePoseFar,points.humanPlayerControlPose,points.humanPlayer))
                .setLinearHeadingInterpolation(points.scorePoseFar.getHeading(),points.humanPlayer.getHeading())
                .build();

        grabHumanPlayerTangent = follower.pathBuilder()
                .addPath(new BezierCurve(points.scorePoseFar,points.humanPlayerControlPose,points.humanPlayer))
                .setTangentHeadingInterpolation()
                .build();

        scoreHumanPlayer = follower.pathBuilder()
                .addPath(new BezierCurve(points.humanPlayer,points.humanPlayerControlPose,points.scorePoseFar))
                .setLinearHeadingInterpolation(points.humanPlayer.getHeading(), points.scorePoseFar.getHeading())
                .build();

        // ------------- LEFT OVER BALLS FROM THE GATE ------------------

        grabLeftoverBallsGate = follower.pathBuilder()
                .addPath(new BezierCurve(points.scorePoseFar, points.controlPoseEatLeftoverGate, points.eatLeftoverGate))
                .setTangentHeadingInterpolation()
                .build();

        scoreLeftoverBallsGate = follower.pathBuilder()
                .addPath(new BezierLine(points.eatLeftoverGate, points.scorePoseFar))
                .setLinearHeadingInterpolation(points.eatLeftoverGate.getHeading(), points.scorePoseFar.getHeading())
                .build();
    }

}
