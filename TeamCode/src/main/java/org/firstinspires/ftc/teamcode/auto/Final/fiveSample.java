package org.firstinspires.ftc.teamcode.auto.Final;

//import com.pedropathing.follower.Follower;
//import com.pedropathing.localization.Pose;
//import com.pedropathing.pathgen.BezierLine;
//import com.pedropathing.pathgen.Path;
//import com.pedropathing.pathgen.PathChain;
//import com.pedropathing.pathgen.Point;
//import com.pedropathing.util.Timer;
//import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
//import com.rowanmcalpin.nextftc.core.command.Command;
//import com.rowanmcalpin.nextftc.core.command.groups.SequentialGroup;
//import com.rowanmcalpin.nextftc.core.command.utility.delays.Delay;
//import com.rowanmcalpin.nextftc.pedro.FollowPath;
//import com.rowanmcalpin.nextftc.pedro.PedroOpMode;
//
//import org.firstinspires.ftc.teamcode.auto.AutoCommands;
//import org.firstinspires.ftc.teamcode.auto.subsystems.ElevatorAngleNext;
//import org.firstinspires.ftc.teamcode.auto.subsystems.intake.nextIntakeAngle;
//import org.firstinspires.ftc.teamcode.auto.subsystems.intake.nextIntakeClaw;
//import org.firstinspires.ftc.teamcode.auto.subsystems.nextLift;
//
//import org.firstinspires.ftc.teamcode.auto.pedro.constants.FConstants;
//import org.firstinspires.ftc.teamcode.auto.pedro.constants.LConstants;
//
//@Autonomous(name = "lior ;)")
//public class fiveSample extends PedroOpMode {
//
//    public fiveSample() {
//          super(nextLift.INSTANCE, nextIntakeAngle.INSTANCE, ElevatorAngleNext.INSTANCE, nextIntakeClaw.INSTANCE);
//      }
//    AutoCommands commands = new AutoCommands();
//
//    // THIS IS A VARIABLE DECLARATION
//    nextLift lift = nextLift.INSTANCE;
//    nextIntakeAngle intakeAngle = nextIntakeAngle.INSTANCE;
//    nextIntakeClaw claw = nextIntakeClaw.INSTANCE;
//    ElevatorAngleNext liftAngle = ElevatorAngleNext.INSTANCE;
//
//
//    private Timer pathTimer, actionTimer, opmodeTimer;
//    private int pathState;
//    private final Pose startPose = new Pose(8.7,104 ,Math.toRadians(0));
//    private final Pose scorePose = new Pose(15,120,Math.toRadians(-45)); // basket
//    private final Pose scoreControl = new Pose(20,115);
//    private final Pose sample1 = new Pose(15,130,0);
//    private final Pose parkPose = new Pose(10, 15.5, Math.toRadians(90));    // Parking position
//
//    private Path park;
//    private PathChain  scorePreload, grabPickup1, take2, take3, scorePickup1, score2, score3;
//
//    public void buildPaths() {
//
//        scorePreload = follower.pathBuilder()
//                .addPath(new BezierLine(new Point(startPose), new Point(scorePose)))
//                .setLinearHeadingInterpolation(startPose.getHeading(), scorePose.getHeading())
//                .build();
////
////
//        grabPickup1 = follower.pathBuilder()
//                .addPath(new BezierLine(new Point(scorePose), new Point(sample1)))
//                .setLinearHeadingInterpolation(scorePose.getHeading(), sample1.getHeading())
//                .build();
//        scorePickup1 = follower.pathBuilder()
//                .addPath(new BezierLine(new Point(sample1), new Point(scorePose)))
//                .setLinearHeadingInterpolation(sample1.getHeading(), scorePose.getHeading())
//                .build();
//        park = new Path(new BezierLine(new Point(startPose), new Point(parkPose)));
//        park.setLinearHeadingInterpolation(startPose.getHeading(), parkPose.getHeading());
//
//    }
//
//
//    public Command preload()  {
//        return new SequentialGroup(
//                new FollowPath(scorePreload),
////                nextIntakeAngle.INSTANCE.Down(),
//                ElevatorAngleNext.INSTANCE.toAngle(1600, 1),
//                new Delay(1),
//                nextLift.INSTANCE.toHeight(2300,1),
//                new Delay(1),
//                nextIntakeAngle.INSTANCE.Up(),
//                new Delay(0.5),
//                nextIntakeClaw.INSTANCE.out(1.5)
//                );
//    }
//
//    public Command testing(){
//        return new SequentialGroup(
//                nextIntakeClaw.INSTANCE.out(2),
//        new FollowPath(park)
//
//        );
//    }
//
//
//
//@Override
//public void onInit() {
//    follower = new Follower(hardwareMap, FConstants.class, LConstants.class);
//    follower.setStartingPose(startPose);
//    buildPaths();
//}
//
//    @Override
//    public void onStartButtonPressed() {
//        preload().invoke();
//    }
//}
