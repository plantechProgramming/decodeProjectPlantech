package org.firstinspires.ftc.teamcode.auto;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.PedroCoordinates;
import com.pedropathing.geometry.Pose;
import com.pedropathing.math.MathFunctions;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;

public class PathsBlue {

    public PathsBlue(){
    }
    // TODO: make stuff private with getter
    private final Pose startPose = new Pose(20.2, 122.5, Math.toRadians(144)); // Start Pose of our robot.
    private final Pose startPoseFar = new Pose(56.5, 8.1, Math.toRadians(180)); // Start Pose of our robot.
    public final Pose scorePoseFar = new Pose(59, 16, Math.toRadians(112)); // Scoring Pose of our robot. It is facing the goal at a 115 degree angle.

    public final Pose scorePose = new Pose(47.5, 95, Math.toRadians(135)); // Scoring Pose of our robot. It is facing the goal at a 135 degree angle.
    public final Pose controlPosePPG = new Pose(70,60);// pose for getting to PPG without hitting other balls
    public final Pose controlPosePGP = new Pose(58,58);// pose for getting to PGP without hitting other balls
    public final Pose controlPoseGPP = new Pose(50,60);// pose for getting to PGP without hitting other balls
    public final Pose controlPoseGate = new Pose(31.5,57.5);

    public final Pose controlPoseGatePPG = new Pose(50, 63); // pose for getting from gate to score without hitting ppg

    public final Pose leaveClosePose = new Pose(55, 122.5, Math.toRadians(167));
    public final Pose leaveFarPose = new Pose(39, 16, Math.toRadians(180));
    public final Pose GPP = new Pose(44, 36, Math.toRadians(180));
    public final Pose PPG = new Pose(44, 83.5, Math.toRadians(180));
    public final Pose PGP = new Pose(44 , 58.5, Math.toRadians(180));

    public final Pose afterPickupGPP = new Pose(11.5, 36, Math.toRadians(180));
    public final Pose afterPickupPPG = new Pose(18, 83.5, Math.toRadians(180));
    public final Pose afterPickupPGP = new Pose(11.5, 58.5, Math.toRadians(180));
    public final Pose gate = new Pose(18,66,Math.toRadians(180));

    public PathChain scorePreload, scorePreloadFar;
    public PathChain grabGPP, grabPGP, grabPPG;
    public PathChain scoreGPP, scorePGP, scorePPG;
    public PathChain grabGPPFar, grabPGPFar;
    public PathChain scoreGPPFar;
    public PathChain gatePGP, scoreGateClose, gateLeaveScoreClose;
    public PathChain scoreLeaveClose, scoreLeaveFar, leaveClose, leaveFar;


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

        scoreLeaveFar = follower.pathBuilder()
                .addPath(new BezierLine(scorePose, leaveFarPose))
                .setLinearHeadingInterpolation(scorePose.getHeading(), leaveFarPose.getHeading())
                .build();

        leaveFar = follower.pathBuilder()
                .addPath(new BezierLine(startPose, leaveFarPose))
                .setLinearHeadingInterpolation(startPose.getHeading(), leaveFarPose.getHeading())
                .build();

        //-------------GATE------------------

        gatePGP = follower.pathBuilder()
                .addPath(new BezierCurve(afterPickupPGP,controlPoseGate,gate))
                .setLinearHeadingInterpolation(afterPickupPGP.getHeading(), gate.getHeading())
                .build();

        scoreGateClose = follower.pathBuilder()
                .addPath(new BezierCurve(gate, controlPoseGatePPG,scorePose))
                .setLinearHeadingInterpolation(gate.getHeading(), scorePose.getHeading())
                .build();

        gateLeaveScoreClose = follower.pathBuilder()
                .addPath(new BezierLine(gate, leaveClosePose))
                .setLinearHeadingInterpolation(gate.getHeading(), leaveClosePose.getHeading())
                .build();
    }

    // these work ONLY FOR BLUE
//    PathChain scorePath(Pose startPose){
//        return follower.pathBuilder()
//                .addPath(new BezierLine(startPose, scorePose))
//                .setLinearHeadingInterpolation(startPose.getHeading(), scorePose.getHeading())
//                .build();
//    }
//
//    PathChain intakePath(Pose startPose, double distance){
//        Pose endPose = new Pose(startPose.getX() - distance,startPose.getY(), startPose.getHeading());
//        return follower.pathBuilder()
//                .addPath(new BezierLine(startPose,endPose))
//                .build();
//    }


}