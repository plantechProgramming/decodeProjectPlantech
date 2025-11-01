package org.firstinspires.ftc.teamcode.auto.test;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.auto.subsystems.NextShooter;

import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.groups.SequentialGroup;
import dev.nextftc.core.components.SubsystemComponent;
import dev.nextftc.ftc.NextFTCOpMode;
import dev.nextftc.ftc.components.BulkReadComponent;

@Autonomous(name = "NextFTC testing")
public class testNext extends NextFTCOpMode {


    public testNext() {
        addComponents(
                new SubsystemComponent(NextShooter.INSTANCE),
                BulkReadComponent.INSTANCE
        );

    }

    private Command autonomousRoutine() {
        return new SequentialGroup(
                NextShooter.INSTANCE.shootByAngle()
        );
    }
    @Override
    public void onStartButtonPressed() {
        autonomousRoutine().schedule();
    }

}