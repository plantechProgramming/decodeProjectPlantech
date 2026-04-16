package org.firstinspires.ftc.teamcode.auto.autos.redAutos;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import dev.nextftc.ftc.NextFTCOpMode;

@Autonomous(name = "Megido Auto Close Red", group = "Red")
public class MegidoAuto extends NextFTCOpMode {
//    private Follower follower;
//
//    public MegidoAuto() {
//        addComponents(
////                new SubsystemComponent(NextShooter.INSTANCE, NextInBetween.INSTANCE),
//                new PedroComponent(Constants::createFollower),
//                AutoCommands.INSTANCE_RED
//        );
//    }
//
//    AutoCommands command = AutoCommands.INSTANCE_RED;
//    PathsRed path;
//
//
//    private final Pose startPose = new Pose(20.1, 122.5, Math.toRadians(144)).mirror(); // Start Pose of our robot.
//
//
//    public Command autoRoutine(){
//        return new SequentialGroup(
//                command.startShooter(false),
//                command.score(path.scorePreload),
//                command.intake(path.grabPPG),
//
//                command.score(path.scorePPG),
//                command.intake(path.grabPGP),
//
//                command.score(path.scorePGP),
//                new FollowPath(path.scoreLeaveClose)
//        );
//    }
//    @Override
//    public void onUpdate(){
//        telemetry.addData("x", follower().getPose().getX());
//        telemetry.addData("y", follower().getPose().getY());
//        telemetry.addData("heading", follower().getPose().getHeading());
//        telemetry.update();
//    }
//    @Override
//    public void onStartButtonPressed() {
//        path = new PathsRed();
//        follower().setStartingPose(path.getSPose());
//        path.buildPaths(follower());
//        autoRoutine().schedule();
//    }
//
//    @Override
//    public void onStop(){
//        ReadWrite readWrite = new ReadWrite();
//        readWrite.writePose(follower().getPose());
//    }
}
