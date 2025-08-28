package org.firstinspires.ftc.teamcode.auto.subsystems;

import com.qualcomm.robotcore.hardware.CRServo;
import com.rowanmcalpin.nextftc.core.Subsystem;
import com.rowanmcalpin.nextftc.core.command.Command;
import com.rowanmcalpin.nextftc.core.command.utility.LambdaCommand;
import com.rowanmcalpin.nextftc.core.control.controllers.Controller;
import com.rowanmcalpin.nextftc.ftc.hardware.controllables.Controllable;

import java.util.Collections;
import java.util.Set;

public class HoldPosition{
    public static Command hold(Controllable controllable, Controller controller) {
        return new LambdaCommand()
                .setStart(() -> {
                    controller.setTarget(controllable.getCurrentPosition());
                    controller.reset();
                })
                .setUpdate(() -> {
                    controllable.setPower(controller.calculate(controllable.getCurrentPosition()));
                })
                .setStop(interrupted -> {
                    controllable.setPower(0.0);
                })
                .setIsDone(() -> false); // Returns if the command has finished


    }

}

//import com.rowanmcalpin.nextftc.core.Subsystem;
//import com.rowanmcalpin.nextftc.core.command.Command;
//import com.rowanmcalpin.nextftc.core.control.controllers.Controller;
//import com.rowanmcalpin.nextftc.ftc.hardware.controllables.Controllable;
//
//import java.util.Collections;
//import java.util.Set;
//
//public class HoldPosition extends Command {
//
//    private final Controllable controllable;
//    private final Controller controller;
//    private final Set<Subsystem> subsystems;
//
//    // Main constructor
//    public HoldPosition(Controllable controllable, Controller controller, Set<Subsystem> subsystems) {
//        this.controllable = controllable;
//        this.controller = controller;
//        this.subsystems = subsystems;
//    }
//
//    // Overloaded constructor for a single subsystem
//    public HoldPosition(Controllable controllable, Controller controller, Subsystem subsystem) {
//        this(controllable, controller, Collections.singleton(subsystem));
//    }
//
//    // Overloaded constructor with no subsystems
//    public HoldPosition(Controllable controllable, Controller controller) {
//        this(controllable, controller, Collections.emptySet());
//    }
//
//    @Override
//    public boolean isDone() {
//        return false;
//    }
//
//    @Override
//    public void start() {
//        controller.setTarget(controllable.getCurrentPosition());
//        controller.reset();
//    }
//
//    @Override
//    public void update() {
//        controllable.setPower(controller.calculate(controllable.getCurrentPosition()));
//    }
//
//    @Override
//    public Set<Subsystem> getSubsystems() {
//        return subsystems;
//    }
//}