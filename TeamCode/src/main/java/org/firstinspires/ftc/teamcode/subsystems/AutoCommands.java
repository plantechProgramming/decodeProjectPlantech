package org.firstinspires.ftc.teamcode.subsystems;

import static com.pedropathing.ivy.commands.Commands.infinite;
import static com.pedropathing.ivy.commands.Commands.instant;
import static com.pedropathing.ivy.commands.Commands.lazy;
import static com.pedropathing.ivy.commands.Commands.waitMs;
import static com.pedropathing.ivy.groups.Groups.parallel;
import static com.pedropathing.ivy.groups.Groups.sequential;
import static com.pedropathing.ivy.pedro.PedroCommands.follow;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.ivy.Command;
import com.pedropathing.paths.PathChain;

import org.firstinspires.ftc.teamcode.Misc.Utils.PoseFunctions;
import org.firstinspires.ftc.teamcode.auto.autos.paths.Paths;
import org.firstinspires.ftc.teamcode.subsystems.Camera.Limelight;

public class AutoCommands{
    public Shooter shooter;
    public Intake intake;
    public InBetween inBetween;
    Follower follower;
    Limelight limelight;

    public AutoCommands(Follower follower) {
        shooter = new Shooter();
        intake = new Intake();
        inBetween = new InBetween();
        this.follower = follower;
        limelight = new Limelight();
    }

    public AutoCommands(){
        shooter = new Shooter();
        intake = new Intake();
        inBetween = new InBetween();
        limelight = new Limelight();
    }

    public Command periodic(){
        return parallel(shooter.periodic());
    }

    private Command goToBlob(){
        Pose targetArtifact;
        try{
            targetArtifact = PoseFunctions.pose2DToPose(
                            limelight.getBestPoseForPickup(PoseFunctions.poseToPose2D(follower.getPose())));
        }
        catch (NullPointerException e){
            return null;
        }
        PathChain path = follower.pathBuilder()
                .addPath(new BezierLine(follower.getPose(), targetArtifact))
                .setTangentHeadingInterpolation()
                .setTValueConstraint(0.5)
                .build();

        return intake(path);

    }
    Command goToArtifact = null;
    public Command goToDetectedBlob(){
        goToArtifact = null;
        return sequential(
                lazy(() -> infinite(() -> goToArtifact = goToBlob()).until(() -> goToArtifact != null)),
                lazy(() -> goToArtifact)
        );
    }

    public Command scoreDetectedBlob(Paths path){ // far score
        return lazy(() -> score(follower.pathBuilder().addPath(new BezierLine(follower.getPose(), path.points.scorePoseFar))
                        .setLinearHeadingInterpolation(follower.getHeading(), path.points.scorePoseFar.getHeading()).build()));
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
        int delay = 500;
        if(far){
            delay = 1200;
        }
        return sequential(
                shooter.naiveShooter(far),
                waitMs(delay)
//                intake.take(),
//                inBetween.inBetweenInPart()
        );
    }

}


