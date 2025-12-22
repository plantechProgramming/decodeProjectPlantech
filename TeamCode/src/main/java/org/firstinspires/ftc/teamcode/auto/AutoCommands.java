package org.firstinspires.ftc.teamcode.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.auto.subsystems.NextInBetween;
import org.firstinspires.ftc.teamcode.auto.subsystems.NextIntake;
import org.firstinspires.ftc.teamcode.auto.subsystems.NextShooter;
import org.firstinspires.ftc.teamcode.teleOp.actions.GetVelocity;

import java.nio.channels.NetworkChannel;

import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.delays.Delay;
import dev.nextftc.core.commands.groups.ParallelGroup;
import dev.nextftc.core.commands.groups.SequentialGroup;
import dev.nextftc.core.components.SubsystemComponent;
import dev.nextftc.ftc.NextFTCOpMode;
import dev.nextftc.ftc.components.BulkReadComponent;
import dev.nextftc.hardware.impl.MotorEx;

public class AutoCommands extends NextFTCOpMode{
    int threshold = 250;
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
        return new ParallelGroup( // take just moves intake motor, so should be parallel
                NextIntake.INSTANCE.take(),
                NextInBetween.INSTANCE.inBetweenIn()
             );
    }

    public Command fullShoot(boolean far){
        return new SequentialGroup(
                NextShooter.INSTANCE.naiveShooter(far),
                new Delay(2),
                NextInBetween.INSTANCE.inBetweenIn(),
                new Delay(1),
                NextIntake.INSTANCE.take()
//                new ParallelGroup(
//                        NextInBetween.INSTANCE.inBetweenIn().perpetually().endAfter(25),
//                        NextShooter.INSTANCE.naiveShooter(far).perpetually().endAfter(25)
//                        )
                );
    }
}


