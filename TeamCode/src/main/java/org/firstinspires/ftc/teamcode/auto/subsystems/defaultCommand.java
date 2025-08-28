package org.firstinspires.ftc.teamcode.auto.subsystems;

import com.rowanmcalpin.nextftc.core.Subsystem;
import com.rowanmcalpin.nextftc.core.command.Command;
import com.rowanmcalpin.nextftc.core.command.CommandManager;
import com.rowanmcalpin.nextftc.core.command.utility.LambdaCommand;
import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.concurrent.atomic.AtomicBoolean;
import java.util.concurrent.atomic.AtomicInteger;

public class defaultCommand extends Subsystem{
    public static final defaultCommand INSTANCE = new defaultCommand();
    Telemetry telemetry;
    public Command defaultCommand(Subsystem subsystem){
//        if(!CommandManager.INSTANCE.hasCommandsUsing(subsystem)){
//            return subsystem.getDefaultCommand();
//        }
//        else{
//
//        }
        // wtf is atomic boolean??
        //AtomicBoolean defaultExists = new AtomicBoolean(false);
        Command subsystemDefault = subsystem.getDefaultCommand();
        return new LambdaCommand()
//                .setStart(() -> {
//                })
                .setUpdate(() -> {
                    if(!CommandManager.INSTANCE.hasCommandsUsing(subsystem)){
                        CommandManager.INSTANCE.scheduleCommand(subsystemDefault);
                        //defaultExists.set(true);
//                        CommandManager.INSTANCE.scheduleCommands();
                    }
                    else {
                        CommandManager.INSTANCE.cancelCommand(subsystemDefault);
                        if (!CommandManager.INSTANCE.hasCommandsUsing(subsystem)){
                            CommandManager.INSTANCE.scheduleCommand(subsystemDefault);
                        }
                        //defaultExists.set(false);
                    }
                })
                .setIsDone(() ->false); // Returns if the command has finished

    }
}
