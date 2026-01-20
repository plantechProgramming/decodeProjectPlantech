package org.firstinspires.ftc.teamcode.auto.autos.blueAutos; // make sure this aligns with class location
import static dev.nextftc.extensions.pedro.PedroComponent.follower;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.auto.AutoCommands;
import org.firstinspires.ftc.teamcode.auto.PathsBlue;
import org.firstinspires.ftc.teamcode.auto.pedro.constants.Constants;

import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.delays.Delay;
import dev.nextftc.core.commands.groups.SequentialGroup;
import dev.nextftc.extensions.pedro.PedroComponent;
import dev.nextftc.ftc.NextFTCOpMode;

@Autonomous(name = "AutoFullFarBlue")
public class AutoFullFar extends NextFTCOpMode {
    private Follower follower;

    public AutoFullFar() {
        addComponents(
//                new SubsystemComponent(NextShooter.INSTANCE, NextInBetween.INSTANCE),
                new PedroComponent(Constants::createFollower),
                AutoCommands.INSTANCE
        );
    }

    AutoCommands command = AutoCommands.INSTANCE;
    PathsBlue path;
    private Timer pathTimer, actionTimer, opmodeTimer;
    private int pathState;


    public final Pose startPose = new Pose(62, 8, Math.toRadians(90)); // Start Pose of our robot.
//    private final Pose scorePose = new Pose(47.60172591970307, 95.1073798180677, Math.toRadians(135)); // Scoring Pose of our robot. It is facing the goal at a 135 degree angle.
//    private final Pose controlPose = new Pose(50,60);// pose for getting to GPP without hitting other balls
//    private final Pose GPP = new Pose(40, 35, Math.toRadians(180));
//    private final Pose PPG = new Pose(40, 85, Math.toRadians(180));
//    private final Pose PGP = new Pose(40, 60, Math.toRadians(180));
//
//    private final Pose afterPickup1 = new Pose(15, 35, Math.toRadians(180));
//
//    private final Pose afterPickup2 = new Pose(15, 84.3, Math.toRadians(180));
//    private final Pose afterPickup3 = new Pose(15, 59, Math.toRadians(180));
//    private final Pose autoEndPose = new Pose(15,59,Math.toRadians(180));


    // Highest (First Set) of Artifacts from the Spike Mark.
//    private final Pose pickup2Pose = new Pose(43, 130, Math.toRadians(0)); // Middle (Second Set) of Artifacts from the Spike Mark.
//    private final Pose pickup3Pose = new Pose(49, 135, Math.toRadians(0)); // Lowest (Third Set) of Artifacts from the Spike Mark.
    private Path scorePreloadFar;
    private PathChain grabGPP, intakeGPP, scoreGPP,intake2, intake3, grabPPG, scorePGP, grabPGP, autoEnd;

    public void buildPaths() {
        /* This is our scorePreloadClose path. We are using a BezierLine, which is a straight line. */
//        scorePreloadFar = new Path(new BezierLine(startPose, scorePoseFar));
//        scorePreloadFar.setLinearHeadingInterpolation(startPose.getHeading(), scorePoseFar.getHeading());

    /* Here is an example for Constant Interpolation
    scorePreloadClose.setConstantInterpolation(startPose.getHeading()); */

        /* This is our grabPickup1 PathChain. We are using a single path with a BezierLine, which is a straight line. */




    }





//    public Command preload_1(){
//        return new SequentialGroup(
//        new FollowPath(scorePreloadClose),
        ////        NextShooter.INSTANCE.naiveShooter(false).invoke()
//        );
//    }
        public Command autoRoutine(){
            path.buildPaths();
            return new SequentialGroup(
                    command.startShooter(true),
                    new Delay(0.3),
                    command.score(path.scorePreloadFar),
                    command.intake(path.intakeGPP, path.grabGPP, 0.70),
                    command.score(path.scoreGPPFar),
                    command.startShooter(false),

                    command.intake(path.intakePGP,path.grabPGP,0.5),
                    new Delay(0.3),
                    command.score(path.scorePGP),
                    command.intake(path.intakePPG,path.grabPPG,0.72)



            );
        }
    @Override
    public void onStartButtonPressed() {
//        Caching caching = new Caching(0.01,);
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startPose);
        path = new PathsBlue(follower());
        path.buildPaths();
        autoRoutine().schedule();
    }

    }

    /** These change the states of the paths and actions. It will also reset the timers of the individual switches **/
