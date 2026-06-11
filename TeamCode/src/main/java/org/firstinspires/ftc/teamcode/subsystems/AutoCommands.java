package org.firstinspires.ftc.teamcode.subsystems;

import static com.pedropathing.ivy.commands.Commands.waitMs;
import static com.pedropathing.ivy.groups.Groups.parallel;
import static com.pedropathing.ivy.groups.Groups.sequential;
import static com.pedropathing.ivy.pedro.PedroCommands.follow;

import com.pedropathing.follower.Follower;
import com.pedropathing.ivy.Command;
import com.pedropathing.paths.PathChain;

public class AutoCommands{
    Shooter shooter;
    Intake intake;
    InBetween inBetween;
    Follower follower;

    public AutoCommands(Follower follower) {
        shooter = new Shooter();
        intake = new Intake();
        inBetween = new InBetween();
        this.follower = follower;
    }

    public AutoCommands(){}

    public void periodic(){
        shooter.periodic();
    }

    public Command shoot(){
        return sequential(
                inBetween.inFull(),
                intake.take()
        );
    }

    public Command take(){
        return parallel(
                inBetween.inPart(),
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

    public Command out(){ // including the shooter
        return parallel(
                partialOut(),
                shooter.out()
        );
    }

    public Command partialOut(){ // not including the shooter
        return parallel(
                intake.out(),
                inBetween.out()
        );
    }

    public Command stopAll(){ // not including the shooter
        return parallel(
                intake.stop(),
                inBetween.stop()
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
                take(),
                follow(follower, grabPath),
                stopAll()
        );
    }
    public Command intakeWithSpeed(PathChain grabPath, double speed){
        return sequential(
                take(),
                follow(follower, grabPath, speed),
                stopAll()
        );
    }

    public Command intakeAndShoot(PathChain grabAndShootPath){
        return sequential(
                take(),
                follow(follower, grabAndShootPath),
                shoot(),
                waitMs(1600)
        );
    }

    public Command startShooter(boolean far){
        return parallel(
                shooter.naiveShooter(far)
//                intake.take(),
//                inBetween.inBetweenInPart()
        );
    }

}


