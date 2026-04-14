package org.firstinspires.ftc.teamcode.auto;

import static com.pedropathing.ivy.commands.Commands.waitMs;
import static com.pedropathing.ivy.groups.Groups.parallel;
import static com.pedropathing.ivy.groups.Groups.sequential;
import static com.pedropathing.ivy.pedro.PedroCommands.follow;

import com.pedropathing.follower.Follower;
import com.pedropathing.ftc.PoseConverter;
import com.pedropathing.geometry.CoordinateSystem;
import com.pedropathing.geometry.Pose;
import com.pedropathing.ivy.CommandBuilder;
import com.pedropathing.ivy.Command;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.auto.subsystems.NextInBetween;
import org.firstinspires.ftc.teamcode.auto.subsystems.NextIntake;
import org.firstinspires.ftc.teamcode.auto.subsystems.NextShooter;
import org.firstinspires.ftc.teamcode.auto.subsystems.NextTurret;
import org.firstinspires.ftc.teamcode.teleOp.Utils;
import org.firstinspires.ftc.teamcode.teleOp.actions.GetVelocity;

import java.nio.channels.NetworkChannel;
import java.security.PublicKey;

import dev.nextftc.core.commands.delays.Delay;
import dev.nextftc.core.commands.groups.ParallelGroup;
import dev.nextftc.core.commands.groups.SequentialGroup;
import dev.nextftc.core.components.Component;
import dev.nextftc.core.components.SubsystemComponent;
import dev.nextftc.extensions.pedro.FollowPath;
import dev.nextftc.ftc.NextFTCOpMode;
import dev.nextftc.ftc.components.BulkReadComponent;
import dev.nextftc.hardware.impl.MotorEx;

public class AutoCommands{
    NextShooter shooter;
    NextIntake intake;
    NextInBetween inBetween;
    Follower follower;
    Utils util = new Utils();

    public AutoCommands(Follower follower, HardwareMap hardwareMap) {
        shooter = new NextShooter(hardwareMap);
        intake = new NextIntake(hardwareMap);
        inBetween = new NextInBetween(hardwareMap);
        this.follower = follower;
    }


    public Command shoot(){
        return sequential(
                inBetween.inBetweenInFull(),
                intake.take()
        );
    }

    public Command score(PathChain path){
        return sequential(
                follow(follower, path),
                shoot(),
                waitMs(900)
        );
    }

    public Command scoreWithDelay(PathChain path, double delay){
        return sequential(
                follow(follower, path),
                shoot(),
                waitMs(delay)
        );
    }

    public Command intake(PathChain grabPath){
        return sequential(
                inBetween.inBetweenInPart(),
                intake.take(),
                follow(follower, grabPath),
                stopAll()
        );
    }
    public Command intakeWithSpeed(PathChain grabPath, double speed){
        return sequential(
                inBetween.inBetweenInPart(),
                intake.take(),
                follow(follower, grabPath, speed),
                stopAll()
        );
    }

    public Command intakeAndShoot(PathChain grabAndShootPath){
        return sequential(
                inBetween.inBetweenInPart(),
                intake.take(),
                follow(follower, grabAndShootPath),
                shoot(),
                waitMs(1600)
        );
    }

    public Command startShooter(boolean far){
        return sequential(
                shooter.naiveShooter(far)
//                intake.take(),
//                inBetween.inBetweenInPart()
        );
    }

    public Command stopAll(){
        return parallel(
                intake.stop(),
                inBetween.stop()
        );
    }

    public Command take(){
        return parallel(
                inBetween.inBetweenInPart(),
                intake.take()
        );
    }

//    @Override
//    public void postInit(){
//        shooter.stop();
//        inBetween.stop();
//        intake.stop();
//    }

}


