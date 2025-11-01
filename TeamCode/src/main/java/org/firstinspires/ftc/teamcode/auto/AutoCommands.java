package org.firstinspires.ftc.teamcode.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.auto.subsystems.NextShooter;

import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.groups.SequentialGroup;
import dev.nextftc.core.components.SubsystemComponent;
import dev.nextftc.ftc.NextFTCOpMode;
import dev.nextftc.ftc.components.BulkReadComponent;
public class AutoCommands extends NextFTCOpMode{

    public AutoCommands() {
        addComponents(
                new SubsystemComponent(NextShooter.INSTANCE),
                BulkReadComponent.INSTANCE
        );
    }
}


