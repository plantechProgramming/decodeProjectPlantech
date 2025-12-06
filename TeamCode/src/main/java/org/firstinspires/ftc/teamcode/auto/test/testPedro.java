//package org.firstinspires.ftc.teamcode.auto.test;
//import com.arcrobotics.ftclib.command.WaitCommand;
//import com.pedropathing.follower.Follower;
//import com.pedropathing.localization.Pose;
//import com.pedropathing.pathgen.BezierCurve;
//import com.pedropathing.pathgen.BezierLine;
//import com.pedropathing.pathgen.Path;
//import com.pedropathing.pathgen.PathChain;
//import com.pedropathing.pathgen.Point;
//import com.pedropathing.util.Constants;
//import com.pedropathing.util.Timer;
//import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
//
//import com.qualcomm.robotcore.hardware.DcMotor;
//import com.qualcomm.robotcore.hardware.DcMotorEx;
//import com.qualcomm.robotcore.hardware.DcMotorSimple;
//import com.qualcomm.robotcore.eventloop.opmode.OpMode;
//import com.rowanmcalpin.nextftc.core.command.Command;
//import com.rowanmcalpin.nextftc.core.command.groups.ParallelGroup;
//import com.rowanmcalpin.nextftc.core.command.groups.SequentialGroup;
//
//import com.rowanmcalpin.nextftc.core.command.utility.delays.Delay;
//import com.rowanmcalpin.nextftc.core.command.utility.delays.WaitUntil;
//import com.rowanmcalpin.nextftc.pedro.FollowPath;
//import com.rowanmcalpin.nextftc.pedro.PedroOpMode;
////import org.firstinspires.ftc.teamcode.Elevator.intake.nextIntakeAngle;
//import org.firstinspires.ftc.teamcode.*;
//
//import pedroPathing.constants.FConstants;
//import pedroPathing.constants.LConstants;
//
//@Autonomous(name = "lior ;)")
//public class testPedro {
//
//    public testPedro() {
//        super(nextLift.INSTANCE, nextIntakeAngle.INSTANCE, ElevatorAngleNext.INSTANCE, nextIntakeClaw.INSTANCE);
//    }
//    AutoCommands commands = new AutoCommands();}