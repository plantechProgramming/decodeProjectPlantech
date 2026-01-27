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
    private final Pose startPoseFar = pathsBlue.getSPoseFar().mirror(); // Start Pose of our robot.
    private final Pose scorePoseFar = pathsBlue.scorePoseFar.mirror(); // Scoring Pose of our robot. It is facing the goal at a 115 degree angle.

    public final Pose scorePose = Mymirror(pathsBlue.scorePose); // Scoring Pose of our robot. It is facing the goal at a 135 degree angle.
    public final Pose controlPosePPG = pathsBlue.controlPosePPG.mirror();// pose for getting to PPG without hitting other balls
    public final Pose controlPosePGP = pathsBlue.controlPosePGP.mirror();// pose for getting to PGP without hitting other balls
    public final Pose controlPoseGPP = pathsBlue.controlPoseGPP.mirror();// pose for getting to PGP without hitting other balls


    public final Pose leaveClosePose = pathsBlue.leaveClosePose.mirror();
    public final Pose GPP = Mymirror(pathsBlue.GPP);
    public final Pose PPG = Mymirror(pathsBlue.PPG);
    public final Pose PGP = Mymirror(pathsBlue.PGP);

    public final Pose afterPickupGPP = Mymirror(pathsBlue.afterPickupGPP);
    public final Pose afterPickupPPG = Mymirror(pathsBlue.afterPickupPPG);
    public final Pose afterPickupPGP = Mymirror(pathsBlue.afterPickupPGP);

//    public final Pose GPP = pathsBlue.GPP.mirror();
//    public final Pose PPG = pathsBlue.PPG.mirror();
//    public final Pose PGP = pathsBlue.PGP.mirror();
//
//    public final Pose afterPickupGPP = pathsBlue.afterPickupGPP.mirror();
//    public final Pose afterPickupPPG = pathsBlue.afterPickupPPG.mirror();
//    public final Pose afterPickupPGP = pathsBlue.afterPickupPGP.mirror();


    public PathChain scorePreload;
    public PathChain scorePreloadFar;
    public PathChain grabGPP, grabPGP, grabPPG;
    public PathChain scoreGPP, scorePGP, scorePPG;
    public PathChain intakeGPP, intakePGP, intakePPG;
    public PathChain grabGPPFar, grabPGPFar;
    public PathChain scoreGPPFar;

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
                .addPath(new BezierLine(this.startPose, this.scorePose))
                .setLinearHeadingInterpolation(startPose.getHeading(), scorePose.getHeading())
                .build();
        scorePreloadFar =  follower.pathBuilder()
                .addPath(new BezierLine(this.startPoseFar, this.scorePoseFar))
                .setLinearHeadingInterpolation(this.startPoseFar.getHeading(), this.scorePoseFar.getHeading())
                .build();

        grabGPP = follower.pathBuilder()
                .addPath(new BezierCurve(this.scorePose, this.controlPosePPG, this.GPP))
                .setLinearHeadingInterpolation(this.scorePose.getHeading(), this.GPP.getHeading())
                .build();

        intakeGPP = follower.pathBuilder()
                .addPath(new BezierLine(this.GPP, this.afterPickupGPP))
                .build();


        scoreGPP = follower.pathBuilder()
                .addPath(new BezierCurve(this.afterPickupGPP, this.controlPoseGPP, this.scorePose))
                .setLinearHeadingInterpolation(this.afterPickupGPP.getHeading(), this.scorePose.getHeading())
                .build();

        grabPPG = follower.pathBuilder()

                .addPath(new BezierLine(this.scorePose, this.PPG))
                .setLinearHeadingInterpolation(this.scorePose.getHeading(), this.PPG.getHeading())
                .build();

//        grabPPGFar = follower.pathBuilder()
//                .addPath(new BezierLine(scorePose, PPG))
//                .setLinearHeadingInterpolation(scorePose.getHeading(), PPG.getHeading())
//                .build();

        intakePPG = follower.pathBuilder()
                .addPath(new BezierLine(this.PPG, this.afterPickupPPG))
                .build();

        scorePPG = follower.pathBuilder()
                .addPath(new BezierLine(this.afterPickupPPG, this.scorePose))
                .setLinearHeadingInterpolation(this.afterPickupPPG.getHeading(), this.scorePose.getHeading())
                .build();

        grabPGP = follower.pathBuilder()
                .addPath(new BezierLine(this.scorePose, this.PGP))
                .setLinearHeadingInterpolation(this.scorePose.getHeading(), this.PGP.getHeading())
                .build();

        grabPGPFar = follower.pathBuilder()
                .addPath(new BezierLine(this.scorePoseFar, this.PGP))
                .setLinearHeadingInterpolation(this.scorePose.getHeading(), this.PGP.getHeading())
                .build();

        scorePGP = follower.pathBuilder()
                .addPath(new BezierCurve(this.afterPickupPGP, this.controlPosePGP, this.scorePose))
                .setLinearHeadingInterpolation(this.PGP.getHeading(),this.scorePose.getHeading())
                .build();

        intakePGP = follower.pathBuilder()
                .addPath(new BezierLine(this.PGP, this.afterPickupPGP))
                .build();

        leaveClose = follower.pathBuilder()
                .addPath(new BezierLine(this.scorePose, this.leaveClosePose))
                .setLinearHeadingInterpolation(this.scorePose.getHeading(), this.leaveClosePose.getHeading())
                .build();
        grabGPPFar = follower.pathBuilder()
                .addPath(new BezierLine(this.scorePoseFar, this.GPP))
                .setLinearHeadingInterpolation(this.scorePoseFar.getHeading(), this.GPP.getHeading())
                .build();

        scoreGPPFar = follower.pathBuilder()
                .addPath(new BezierLine(this.afterPickupGPP, this.scorePoseFar))
                .setLinearHeadingInterpolation(this.afterPickupGPP.getHeading(), this.scorePoseFar.getHeading())
                .build();

    }
}