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
    String team;
    public static final AutoCommands INSTANCE_RED = new AutoCommands("RED");
    public static final AutoCommands INSTANCE_BLUE = new AutoCommands("BLUE");
    Utils util = new Utils();

    public AutoCommands(Follower follower) {
        shooter = new NextShooter();
        intake = new NextIntake();
        inBetween = new NextInBetween();
        turret = new NextTurret();
        this.follower = follower;
    }

    public AutoCommands(String team){
        shooter = new NextShooter();
        intake = new NextIntake();
        inBetween = new NextInBetween();
        turret = new NextTurret();
        this.team = team;
    }


    public Command shoot(){
        return new SequentialGroup(
                inBetween.inBetweenInFull(),
                intake.take()
        );
    }

    public Command score(PathChain path){
        return new SequentialGroup(
                new FollowPath(path),
                shoot(),
                new Delay(1.6)
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

    public Command startShooter(boolean far){
        return new SequentialGroup(
                shooter.naiveShooter(far),
                intake.take(),
                inBetween.inBetweenInPart()
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
    public Command turnTurretToGoal(){
        Pose pose = follower.getPose();
        Pose2D ftcPose = util.PedroPoseConverter(pose);
        return turret.turnToDeg(util.getPointToGoalAngle(ftcPose,team));
    }
    @Override
    public void postInit(){
        shooter.stop();
        inBetween.stop();
        intake.stop();
    }

}


