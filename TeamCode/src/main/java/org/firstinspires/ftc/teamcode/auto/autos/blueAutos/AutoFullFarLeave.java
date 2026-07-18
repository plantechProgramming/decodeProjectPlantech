package org.firstinspires.ftc.teamcode.auto.autos.blueAutos; // make sure this aligns with class location
import static com.pedropathing.ivy.commands.Commands.instant;
import static com.pedropathing.ivy.groups.Groups.sequential;
import static com.pedropathing.ivy.pedro.PedroCommands.follow;

import com.pedropathing.follower.Follower;
import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.Scheduler;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Misc.Utils.Alliance;
import org.firstinspires.ftc.teamcode.TeamOpMode;
import org.firstinspires.ftc.teamcode.auto.TeamAuto;
import org.firstinspires.ftc.teamcode.subsystems.AutoCommands;
import org.firstinspires.ftc.teamcode.auto.autos.paths.Paths;
import org.firstinspires.ftc.teamcode.auto.autos.paths.Points;
import org.firstinspires.ftc.teamcode.auto.pedro.Constants;

@Autonomous(name = "Full Far Blue leave", group = "Blue")
public class AutoFullFarLeave extends TeamAuto {

    @Override
    public void postInit() {
        Alliance.set(Alliance.BLUE);
        isFar = true;
    }

    @Override
    public Command autoRoutine() {
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
}