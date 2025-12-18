package org.firstinspires.ftc.teamcode.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.auto.subsystems.NextInBetween;
import org.firstinspires.ftc.teamcode.auto.subsystems.NextIntake;
import org.firstinspires.ftc.teamcode.auto.subsystems.NextShooter;

import java.nio.channels.NetworkChannel;

import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.groups.ParallelGroup;
import dev.nextftc.core.commands.groups.SequentialGroup;
import dev.nextftc.core.components.SubsystemComponent;
import dev.nextftc.ftc.NextFTCOpMode;
import dev.nextftc.ftc.components.BulkReadComponent;
public class AutoCommands extends NextFTCOpMode{

    public AutoCommands() {
        addComponents(
                new SubsystemComponent(NextShooter.INSTANCE, NextInBetween.INSTANCE, NextIntake.INSTANCE),
                BulkReadComponent.INSTANCE
        );
    }
    public Command preload1(){
        return new SequentialGroup(
                NextShooter.INSTANCE.naiveShooter(false)
        );
    }

    public Command intake(){
        return new ParallelGroup(
                NextIntake.INSTANCE.take(),
                NextInBetween.INSTANCE.inBetweenFunc(true,false)
             );
    }

    public Command fullShoot(boolean far){
        return new SequentialGroup(
                NextShooter.INSTANCE.naiveShooter(far),
                new ParallelGroup(
                    NextInBetween.INSTANCE.inBetweenFunc(true, false),
                    NextShooter.INSTANCE.naiveShooter(far)

                        )
            );
    }



}


