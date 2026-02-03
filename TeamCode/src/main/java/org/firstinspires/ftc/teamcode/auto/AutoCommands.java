package org.firstinspires.ftc.teamcode.auto;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.auto.subsystems.NextInBetween;
import org.firstinspires.ftc.teamcode.auto.subsystems.NextIntake;
import org.firstinspires.ftc.teamcode.auto.subsystems.NextShooter;
import org.firstinspires.ftc.teamcode.auto.subsystems.NextTurret;
import org.firstinspires.ftc.teamcode.teleOp.actions.GetVelocity;

import java.nio.channels.NetworkChannel;

import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.delays.Delay;
import dev.nextftc.core.commands.groups.ParallelGroup;
import dev.nextftc.core.commands.groups.SequentialGroup;
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

    public AutoCommands(Follower follower) {
        shooter = new NextShooter();
        intake = new NextIntake();
        inBetween = new NextInBetween();
        turret = new NextTurret();
        this.follower = follower;
    }

    public AutoCommands(){
        shooter = new NextShooter();
        intake = new NextIntake();
        inBetween = new NextInBetween();
        turret = new NextTurret();
    }

    public static final AutoCommands INSTANCE = new AutoCommands();

    public Command shoot(){
        return new SequentialGroup(
                inBetween.inBetweenInFull(),
                new Delay(0.6),
                intake.take()
        );
    }

    public Command score(PathChain path){
        return new SequentialGroup(
                new FollowPath(path),
                shoot(),
                new Delay(2)
        );
    }

    public Command intake(PathChain intakePath,PathChain toGrabPath, double speed){
        return new SequentialGroup(
                new FollowPath(toGrabPath),
                inBetween.inBetweenInPart(),
                intake.take(),
                new FollowPath(intakePath, true, speed),
                stopAll()
        );
    }

    public Command startShooter(boolean far){
        return new SequentialGroup(
                shooter.naiveShooter(far)
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
        Pose pos = follower.getPose();
        Pose2D ftcPose = turret.PedroPoseConverter(pos);
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
        turret.stop();
    }

}


