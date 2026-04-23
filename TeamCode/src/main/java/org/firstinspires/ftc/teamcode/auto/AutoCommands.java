package org.firstinspires.ftc.teamcode.auto;

import static com.pedropathing.ivy.commands.Commands.waitMs;
import static com.pedropathing.ivy.groups.Groups.parallel;
import static com.pedropathing.ivy.groups.Groups.sequential;
import static com.pedropathing.ivy.pedro.PedroCommands.follow;

import com.pedropathing.follower.Follower;
import com.pedropathing.ivy.Command;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.auto.subsystems.NextInBetween;
import org.firstinspires.ftc.teamcode.auto.subsystems.NextIntake;
import org.firstinspires.ftc.teamcode.auto.subsystems.NextShooter;
import org.firstinspires.ftc.teamcode.teleOp.Utils;

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

    public void periodic(){
        shooter.periodic();
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
        return parallel(
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

}


