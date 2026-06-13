package org.firstinspires.ftc.teamcode.auto;

import com.pedropathing.follower.Follower;
import com.pedropathing.ftc.PoseConverter;
import com.pedropathing.geometry.CoordinateSystem;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.seattlesolvers.solverslib.command.ConditionalCommand;

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
import java.util.function.BooleanSupplier;

import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.conditionals.IfElseCommand;
import dev.nextftc.core.commands.delays.Delay;
import dev.nextftc.core.commands.groups.ParallelGroup;
import dev.nextftc.core.commands.groups.SequentialGroup;
import dev.nextftc.core.commands.utility.InstantCommand;
import dev.nextftc.core.components.Component;
import dev.nextftc.core.components.SubsystemComponent;
import dev.nextftc.extensions.pedro.FollowPath;
import dev.nextftc.ftc.NextFTCOpMode;
import dev.nextftc.ftc.components.BulkReadComponent;
import dev.nextftc.hardware.impl.MotorEx;

public class AutoCommands implements Component{
    NextShooter shooter;
    NextIntake intake;
    NextInBetween inBetween;
    NextTurret turret;
    Follower follower;
    VoltageSensor voltageSensor;
    Utils util = new Utils();

    public AutoCommands(Follower follower, VoltageSensor voltageSensor) {
        shooter = new NextShooter(voltageSensor);
        intake = new NextIntake();
        inBetween = new NextInBetween();
        turret = new NextTurret();
        this.follower = follower;
        this.voltageSensor = voltageSensor;
    }



    public Command shoot(){
        return new ParallelGroup(
                inBetween.inBetweenInFull(),
                intake.take()
        );
    }

    public Command score(PathChain path){
        return new SequentialGroup(
                new FollowPath(path),
                shoot(),
                new Delay(0.93)
        );
    }

    public Command scorePreload(PathChain path){
        return new SequentialGroup(
                scoreWithDelay(path, 1)
        );
    }

    public Command scoreWithDelay(PathChain path, double delay){
        return new SequentialGroup(
                new FollowPath(path),
                shoot(),
                new Delay(delay)
        );
    }

    public Command intake(PathChain grabPath){
        return new SequentialGroup(
                inBetween.inBetweenInPart(),
                intake.take(),
                new FollowPath(grabPath),
                stopAll()
        );
    }
    public Command intakeWithSpeed(PathChain grabPath, double speed){
        return new SequentialGroup(
                inBetween.inBetweenInPart(),
                intake.take(),
                new FollowPath(grabPath, true, speed),
                stopAll()
        );
    }

    public Command intakeAndShoot(PathChain grabAndShootPath){
        return new SequentialGroup(
                inBetween.inBetweenInPart(),
                intake.take(),
                new FollowPath(grabAndShootPath),
                shoot(),
                new Delay(1.6)
        );
    }

    public Command startShooter(boolean far){
        double delay = 0;
        if(far){
            delay = 1;
        }
        return new SequentialGroup(
                shooter.naiveShooter(far),
                new Delay(delay),
                inBetween.startAxons(),
                intake.take()
        );
    }

    public Command stopAll(){
        return new ParallelGroup(
                intake.stop(),
                inBetween.stop()
        );
    }

    public Command take(){
        return new ParallelGroup(
                inBetween.inBetweenInPart(),
                intake.take()
        );
    }
    public Command turnTurret(){
        Pose pose = follower.getPose();
        Pose2D ftcPose = util.PedroPoseConverter(pose);
        double heading = ftcPose.getHeading(AngleUnit.DEGREES);
        return turret.turnToDeg(heading);
    }

    @Override
    public void postUpdate(){
        shooter.Periodic();
    }

    @Override
    public void postInit(){
        shooter.stop();
        inBetween.stop();
        intake.stop();
    }

}


