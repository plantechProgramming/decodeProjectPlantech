package org.firstinspires.ftc.teamcode.Tests.Auto;

import static com.pedropathing.ivy.commands.Commands.waitMs;
import static com.pedropathing.ivy.groups.Groups.sequential;
import static com.pedropathing.ivy.pedro.PedroCommands.follow;
import static com.pedropathing.ivy.pedro.PedroCommands.hold;

import com.pedropathing.ivy.Command;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Misc.Utils.Alliance;
import org.firstinspires.ftc.teamcode.auto.TeamAuto;

@Autonomous(group="autonomous tests")
public class test extends TeamAuto {

    @Override
    public void postInit(){
        isFar = true;
        Alliance.set(Alliance.BLUE);
    }

    @Override
    public Command autoRoutine() {
        return sequential(
                follow(follower, path.scorePreloadFar),
                waitMs(1000),
                follow(follower, path.grabGPPFar),
                follow(follower, path.scoreGPPFar)
        );
    }

}
