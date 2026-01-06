//package org.firstinspires.ftc.teamcode.auto;
//
//import com.pedropathing.follower.Follower;
//import com.pedropathing.geometry.BezierCurve;
//import com.pedropathing.geometry.BezierLine;
//import com.pedropathing.geometry.Pose;
//import com.pedropathing.paths.Path;
//import com.pedropathing.paths.PathChain;
//
//public class PathsRed {
//
//    private Follower follower;
//
//    private final Pose startPose = new Pose(111, 135.3, Math.toRadians(0)); // Start Pose of our robot.
//    private final Pose scorePose = new Pose(95, 95, Math.toRadians(45)); // Scoring Pose of our robot. It is facing the goal at a 135 degree angle.
//    private final Pose controlPose = new Pose(70,60);// pose for getting to GPP without hitting other balls
//    private final Pose GPP = new Pose(104, 35, Math.toRadians(0));
//    private final Pose PPG = new Pose(104, 84.3, Math.toRadians(0));
//    private final Pose PGP = new Pose(104, 59, Math.toRadians(0));
//
//    private final Pose afterPickup1 = new Pose(130, 35, Math.toRadians(0));
//
//    private final Pose afterPickup2 = new Pose(130, 84.3, Math.toRadians(0));
//    private final Pose afterPickup3 = new Pose(130, 59, Math.toRadians(0));
//    private final Pose autoEndPose = new Pose(130,59,Math.toRadians(0));
//
//
//    // Highest (First Set) of Artifacts from the Spike Mark.
////    private final Pose pickup2Pose = new Pose(43, 130, Math.toRadians(0)); // Middle (Second Set) of Artifacts from the Spike Mark.
////    private final Pose pickup3Pose = new Pose(49, 135, Math.toRadians(0)); // Lowest (Third Set) of Artifacts from the Spike Mark.
//    private Path scorePreload;
//    private PathChain grabGPP, intake1, scoreGPP,intake2, intake3, grabPPG, scorePPG, grabPGP, autoEnd;
//
//    public void buildPaths() {
//        /* This is our scorePreload path. We are using a BezierLine, which is a straight line. */
//        scorePreload = new Path(new BezierLine(startPose, scorePose));
//        scorePreload.setLinearHeadingInterpolation(startPose.getHeading(), scorePose.getHeading());
//
//    /* Here is an example for Constant Interpolation
//    scorePreload.setConstantInterpolation(startPose.getHeading()); */
//
//        /* This is our grabPickup1 PathChain. We are using a single path with a BezierLine, which is a straight line. */
//        grabGPP = follower.pathBuilder()
//                .addPath(new BezierCurve(scorePose,controlPose, GPP))
//                .setLinearHeadingInterpolation(scorePose.getHeading(), GPP.getHeading())
//                .build();
//
//        intake1 = follower.pathBuilder()
//                .addPath(new BezierLine(GPP, afterPickup1))
//                .build();
//        scoreGPP = follower.pathBuilder()
//                .addPath(new BezierCurve(afterPickup1,controlPose, scorePose))
//                .setLinearHeadingInterpolation(afterPickup1.getHeading(), scorePose.getHeading())
//                .build();
//
//        grabPPG = follower.pathBuilder()
//                .addPath(new BezierLine(scorePose, PPG))
//                .setLinearHeadingInterpolation(scorePose.getHeading(), PPG.getHeading())
//                .build();
//
//        intake2 = follower.pathBuilder()
//                .addPath(new BezierLine(PPG, afterPickup2))
//                .build();
//
//        scorePPG = follower.pathBuilder()
//                .addPath(new BezierLine(afterPickup2, scorePose))
//                .setLinearHeadingInterpolation(afterPickup2.getHeading(), scorePose.getHeading())
//                .build();
//        grabPGP = follower.pathBuilder()
//                .addPath(new BezierLine(scorePose, PGP))
//                .setLinearHeadingInterpolation(scorePose.getHeading(), PGP.getHeading())
//                .build();
//
//        intake3 = follower.pathBuilder()
//                .addPath(new BezierLine(PGP, afterPickup3))
//                .build();
//
//        autoEnd = follower.pathBuilder()
//                .addPath(new BezierLine(afterPickup3, autoEndPose))
//                .setLinearHeadingInterpolation(afterPickup2.getHeading(), scorePose.getHeading())
//                .build();
//    }
//}
