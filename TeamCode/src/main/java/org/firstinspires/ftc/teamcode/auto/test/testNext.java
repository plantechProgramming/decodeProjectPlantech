package org.firstinspires.ftc.teamcode.auto.test;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.auto.subsystems.NextInBetween;
import org.firstinspires.ftc.teamcode.auto.subsystems.NextIntake;
import org.firstinspires.ftc.teamcode.auto.subsystems.NextShooter;

import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.groups.SequentialGroup;
import dev.nextftc.core.components.SubsystemComponent;
import dev.nextftc.ftc.NextFTCOpMode;
import dev.nextftc.ftc.components.BulkReadComponent;
import dev.nextftc.core.commands.CommandManager;

@Autonomous(name = "NextFTC testing")
public class testNext extends NextFTCOpMode {


    public testNext() {
        addComponents(
                new SubsystemComponent(NextShooter.INSTANCE, NextInBetween.INSTANCE),
                BulkReadComponent.INSTANCE
        );

    }

    private Command autonomousRoutine() {
        return new SequentialGroup(
                NextShooter.INSTANCE.naiveShooter(true)
        );
    }
    @Override
    public void onStartButtonPressed() {
        autonomousRoutine().schedule();
    }

}