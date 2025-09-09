package org.firstinspires.ftc.teamcode.auto.test;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.auto.subsystems.nextLift;

import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.delays.Delay;
import dev.nextftc.core.commands.groups.ParallelGroup;
import dev.nextftc.core.commands.groups.SequentialGroup;
import dev.nextftc.core.components.SubsystemComponent;
import dev.nextftc.ftc.NextFTCOpMode;
import dev.nextftc.ftc.components.BulkReadComponent;

@Autonomous(name = "NextFTC Autonomous Program Java")
public class testNext extends NextFTCOpMode {
    public testNext() {
        addComponents(
                new SubsystemComponent(nextLift.INSTANCE),
                BulkReadComponent.INSTANCE
        );
    }

    private Command autonomousRoutine() {
        return new SequentialGroup(
                nextLift.INSTANCE.toHeight(1000, 50)
        );
    }

    @Override
    public void onStartButtonPressed() {
        autonomousRoutine().schedule();
    }
}