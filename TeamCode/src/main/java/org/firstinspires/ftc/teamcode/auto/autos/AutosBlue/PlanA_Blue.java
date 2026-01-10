package org.firstinspires.ftc.teamcode.auto.autos.AutosBlue; // make sure this aligns with class location
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import  com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.auto.pedro.constants.Constants;

import dev.nextftc.extensions.pedro.FollowPath;

@Autonomous(name = "PlanA_Blue")
public class PlanA_Blue extends OpMode {
    private Follower follower;
    private Timer pathTimer, actionTimer, opmodeTimer;
    private int pathState;



    private final Pose startPose = new Pose(19, 121.5, Math.toRadians(144)); // Start Pose of our robot.
    private final Pose scorePose = new Pose(47.60172591970307, 95.1073798180677, Math.toRadians(135)); // Scoring Pose of our robot. It is facing the goal at a 135 degree angle.
    private final Pose controlPose = new Pose(70,60);// pose for getting to GPP without hitting other balls
    private final Pose GPP = new Pose(40, 35, Math.toRadians(180));
    private final Pose PPG = new Pose(40, 84.3, Math.toRadians(180));
    private final Pose PGP = new Pose(40, 59, Math.toRadians(180));

    private final Pose afterPickup1 = new Pose(15, 35, Math.toRadians(180));

    private final Pose afterPickup2 = new Pose(15, 84.3, Math.toRadians(180));
    private final Pose afterPickup3 = new Pose(15, 59, Math.toRadians(180));
    private final Pose autoEndPose = new Pose(15,59,Math.toRadians(180));


    // Highest (First Set) of Artifacts from the Spike Mark.
//    private final Pose pickup2Pose = new Pose(43, 130, Math.toRadians(0)); // Middle (Second Set) of Artifacts from the Spike Mark.
//    private final Pose pickup3Pose = new Pose(49, 135, Math.toRadians(0)); // Lowest (Third Set) of Artifacts from the Spike Mark.
    private Path scorePreload;
    private PathChain grabGPP, intake1, scoreGPP,intake2, intake3, grabPPG, scorePPG, grabPGP, autoEnd;

    public void buildPaths() {
        /* This is our scorePreload path. We are using a BezierLine, which is a straight line. */
        scorePreload = new Path(new BezierLine(startPose, scorePose));
        scorePreload.setLinearHeadingInterpolation(startPose.getHeading(), scorePose.getHeading());

    /* Here is an example for Constant Interpolation
    scorePreload.setConstantInterpolation(startPose.getHeading()); */

        /* This is our grabPickup1 PathChain. We are using a single path with a BezierLine, which is a straight line. */
        grabGPP = follower.pathBuilder()
                .addPath(new BezierCurve(scorePose,controlPose, GPP))
                .setLinearHeadingInterpolation(scorePose.getHeading(), GPP.getHeading())
                .build();

        intake1 = follower.pathBuilder()
                .addPath(new BezierLine(GPP, afterPickup1))
                .build();
        scoreGPP = follower.pathBuilder()
                .addPath(new BezierCurve(afterPickup1,controlPose, scorePose))
                .setLinearHeadingInterpolation(afterPickup1.getHeading(), scorePose.getHeading())
                .build();

        grabPPG = follower.pathBuilder()
                .addPath(new BezierLine(scorePose, PPG))
                .setLinearHeadingInterpolation(scorePose.getHeading(), PPG.getHeading())
                .build();

        intake2 = follower.pathBuilder()
                .addPath(new BezierLine(PPG, afterPickup2))
                .build();

        scorePPG = follower.pathBuilder()
                .addPath(new BezierLine(afterPickup2, scorePose))
                .setLinearHeadingInterpolation(afterPickup2.getHeading(), scorePose.getHeading())
                .build();
        grabPGP = follower.pathBuilder()
                .addPath(new BezierLine(scorePose, PGP))
                .setLinearHeadingInterpolation(scorePose.getHeading(), PGP.getHeading())
                .build();

        intake3 = follower.pathBuilder()
                .addPath(new BezierLine(PGP, afterPickup3))
                .build();

        autoEnd = follower.pathBuilder()
                .addPath(new BezierLine(afterPickup3, autoEndPose))
                .setLinearHeadingInterpolation(afterPickup2.getHeading(), scorePose.getHeading())
                .build();
    }

//    public Command preload_1(){
//        return new SequentialGroup(
//        new FollowPath(scorePreload),
////        NextShooter.INSTANCE.naiveShooter(false).invoke()
//        );
//    }
    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
            follower.followPath(scorePreload);
               setPathState(1);
                break;
            case 1:
                if(!follower.isBusy()) {
                    follower.followPath(grabGPP);

                    setPathState(2);
                }
                break;
            case 2:
                if(!follower.isBusy()) {
                    follower.followPath(intake1);
                    setPathState(3);
                }
                break;
            case 3:
                if(!follower.isBusy()) {
                    /* Score Preload */
                    /* Since this is a pathChain, we can have Pedro hold the end point while we are grabbing the sample */
                    follower.followPath(scoreGPP);
                    setPathState(4);
                }
                break;
            case 4:
                if(!follower.isBusy()){
                    follower.followPath(grabPPG);
                    setPathState(5);
                }
                break;
            case 5:
                if ((!follower.isBusy())){
                    follower.followPath(intake2);
                    setPathState(6);
                }
                break;
            case 6:
                if (!follower.isBusy()){
                    follower.followPath(scorePPG);
                    setPathState(7);
                }break;
            case 7:
                if (!follower.isBusy()){
                    follower.followPath(grabPGP);
                    setPathState(8);
                }break;
            case 8:
                if ((!follower.isBusy())){
                    follower.followPath(intake3);
                    setPathState(9);
                }
                break;
            case 9:
                if ((!follower.isBusy())){
                    follower.followPath(autoEnd,true);
                    setPathState(10);
                }
                break;
            case 10:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */
                if(!follower.isBusy()) {
                    /* Set the state to a Case we won't use or define, so it just stops running an new paths */
                    setPathState(-1);
                }
                break;
        }
    }
    /** These change the states of the paths and actions. It will also reset the timers of the individual switches **/
    public void setPathState(int pState) {
        pathState = pState;
        pathTimer.resetTimer();
    }
    /** This is the main loop of the OpMode, it will run repeatedly after clicking "Play". **/
@Override
public void loop() {

        // These loop the movements of the robot, these must be called continuously in order to work
        follower.update();
        autonomousPathUpdate();

        // Feedback to Driver Hub for debugging
        telemetry.addData("path state", pathState);
        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("heading", follower.getPose().getHeading());
        telemetry.update();
    }

    /** This method is called once at the init of the OpMode. **/
    @Override
    public void init() {
        pathTimer = new Timer();
        opmodeTimer = new Timer();
        opmodeTimer.resetTimer();


        follower = Constants.createFollower(hardwareMap);
        buildPaths();
        follower.setStartingPose(startPose);

    }

    /** This method is called continuously after Init while waiting for "play". **/
    @Override
    public void init_loop() {}

    /** This method is called once at the start of the OpMode.
     * It runs all the setup actions, including building paths and starting the path system **/
    @Override
    public void start() {
        opmodeTimer.resetTimer();
        setPathState(0);
    }

    /** We do not use this because everything should automatically disable **/
    @Override
    public void stop() {}
}