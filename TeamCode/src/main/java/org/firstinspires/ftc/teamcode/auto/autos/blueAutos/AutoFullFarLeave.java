package org.firstinspires.ftc.teamcode.auto.autos.blueAutos; // make sure this aligns with class location
import static com.pedropathing.ivy.groups.Groups.sequential;
import static com.pedropathing.ivy.pedro.PedroCommands.follow;
import static org.firstinspires.ftc.teamcode.auto.pedro.Tuning.follower;

import com.pedropathing.follower.Follower;
import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.Scheduler;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.auto.AutoCommands;
import org.firstinspires.ftc.teamcode.auto.autos.paths.Paths;
import org.firstinspires.ftc.teamcode.auto.autos.paths.Points;
import org.firstinspires.ftc.teamcode.auto.autos.ReadWrite;
import org.firstinspires.ftc.teamcode.auto.pedro.constants.Constants;

@Autonomous(name = "Full Far Blue leave", group = "Blue")
public class AutoFullFarLeave extends LinearOpMode {

    AutoCommands command;
    Paths path;
    Follower follower;
    ReadWrite readWrite = new ReadWrite();

        public Command autoRoutine(){
            return sequential(
                command.startShooter(true),
                command.score(path.scorePreloadFar),

                command.intake(path.grabGPPFar),
                command.score(path.scoreGPPFar),

                command.intakeWithSpeed(path.grabLeftoverBallsGate, 0.6),
                command.score(path.scoreLeftoverBallsGate),

                follow(follower, path.scoreLeaveFar)
            );
        }

    public void onStop(){
        readWrite.writePose(follower.getPose());
    }

    @Override
    public void runOpMode() {
        path = new Paths("BLUE");
        follower = Constants.createFollower(hardwareMap);

        follower.setStartingPose(Points.startPoseFar);
        command = new AutoCommands(follower, hardwareMap);
        path.buildPaths(follower);

        Scheduler.schedule(autoRoutine());

        while (opModeIsActive()) {
            command.periodic();
            Scheduler.execute();
        }
    }
}