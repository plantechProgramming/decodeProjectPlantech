package org.firstinspires.ftc.teamcode.auto.subsystems;

import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.groups.SequentialGroup;
import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.hardware.controllable.Controllable;
import dev.nextftc.hardware.impl.MotorEx;

public class nextLift implements Subsystem {
    MotorEx motorEx = new MotorEx("EH")
            .zeroed()
            .brakeMode();

    @Override
    public void initialize() {
        // initialization logic (runs on init)
    }

    public Command toHeight(double height){


    }

    @Override
    public void periodic() {
        // periodic logic (runs every loop)
    }

//    public static final nextLift INSTANCE = new nextLift();
//    public nextLift() { }
//    public MotorEx motor;
//    String name = "EH";
//    public PIDFController controller = new PIDFController(0.001, 0.02, 0.0, new StaticFeedforward(0.7));
//
////    public String EH = "EH";
//    public Command setTolerance(int tolerance){
//        return new LambdaCommand().setStart(()->{
//            controller.setSetPointTolerance(tolerance);
//        });
//    }
//
//    public Command toHeight(double height, double sec) {
//        return new SequentialGroup(
//                new RunToPosition(motor,height,controller,this),
//                new HoldPosition(motor, controller, this).perpetually().endAfter(sec)
//
//        );
//
//    }
//
//
////    public Command cb() {
////       if(CommandManager.INSTANCE.hasCommands() == False)
////    }
//
//    @Override
//    public void initialize() {
//        motor = new MotorEx(name);
//        motor.resetEncoder();
//        motor.reverse();
//
//        setTolerance(100);
//
//    }
}
