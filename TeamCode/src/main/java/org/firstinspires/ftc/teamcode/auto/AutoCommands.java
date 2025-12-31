package org.firstinspires.ftc.teamcode.auto;

import com.pedropathing.follower.Follower;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.auto.subsystems.NextInBetween;
import org.firstinspires.ftc.teamcode.auto.subsystems.NextIntake;
import org.firstinspires.ftc.teamcode.auto.subsystems.NextShooter;
import org.firstinspires.ftc.teamcode.teleOp.actions.GetVelocity;

import java.nio.channels.NetworkChannel;

import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.delays.Delay;
import dev.nextftc.core.commands.groups.ParallelGroup;
import dev.nextftc.core.commands.groups.SequentialGroup;
import dev.nextftc.core.components.SubsystemComponent;
import dev.nextftc.extensions.pedro.FollowPath;
import dev.nextftc.ftc.NextFTCOpMode;
import dev.nextftc.ftc.components.BulkReadComponent;
import dev.nextftc.hardware.impl.MotorEx;

public class AutoCommands{
    int threshold = 250;
    NextShooter shooter;
    NextIntake intake;
    NextInBetween inBetween;
    Follower follower;
    public AutoCommands(Follower follower) {
        shooter = new NextShooter();
        intake = new NextIntake();
        inBetween = new NextInBetween();
        this.follower = follower;
    }

    public Command shoot(){
        return new SequentialGroup( // take just moves intake motor, so should be parallel
                inBetween.inBetweenInFull(),
                new Delay(1),
                intake.take()
        );
    }

    public Command score(PathChain path){
        return new SequentialGroup(
                new FollowPath(path),
                shoot()
        );
    }

    public Command intake(PathChain intakePath,PathChain toGrabPath, double speed){
        return new SequentialGroup(
                new FollowPath(toGrabPath),
                stopPrimers(),
                new Delay(2),
                new FollowPath(intakePath, true,speed),
                stopAll()
        );
    }
    private Command onStart() {
        return startShooter(false);
    }
    public Command startShooter(boolean far){
        return shooter.naiveShooter(far);

    }

    public Command stopAll(){
        return new ParallelGroup(
                intake.stop(),
                inBetween.stop()
        );
    }
    public Command stopPrimers(){
        return inBetween.stopShooterPrimers();
    }

    public Command take(){
        return new ParallelGroup(
                inBetween.inBetweenInPart(),
                intake.take()
        );
    }
}


