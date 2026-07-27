package org.firstinspires.ftc.teamcode.auto.autos.redAutos;

import static com.pedropathing.ivy.groups.Groups.sequential;
import static com.pedropathing.ivy.pedro.PedroCommands.follow;

import com.pedropathing.ivy.Command;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Misc.Utils.Alliance;
import org.firstinspires.ftc.teamcode.auto.TeamAuto;

@Autonomous(name = "Leave Close PPG Red", group = "Red")
public class LeaveClosePPGRed extends TeamAuto {
    @Override
    public void postInit() {
        Alliance.set(Alliance.RED);
        isFar = false;
    }

    @Override
    public Command autoRoutine() {
        return sequential(
                command.startShooter(false),
                command.score(path.scorePreload),

                command.intake(path.grabPPG),
                command.score(path.scorePPG),

                follow(follower, path.scoreLeaveClose)
        );
    }
}
