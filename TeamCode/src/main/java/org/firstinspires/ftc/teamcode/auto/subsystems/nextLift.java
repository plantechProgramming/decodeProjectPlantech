package org.firstinspires.ftc.teamcode.auto.subsystems;


import com.rowanmcalpin.nextftc.core.Subsystem;
import com.rowanmcalpin.nextftc.core.command.Command;
import com.rowanmcalpin.nextftc.core.command.CommandManager;
import com.rowanmcalpin.nextftc.core.command.groups.ParallelRaceGroup;
import com.rowanmcalpin.nextftc.core.command.groups.SequentialGroup;
import com.rowanmcalpin.nextftc.core.command.utility.LambdaCommand;
import com.rowanmcalpin.nextftc.core.command.utility.delays.Delay;
import com.rowanmcalpin.nextftc.core.control.controllers.PIDFController;
import com.rowanmcalpin.nextftc.core.control.controllers.feedforward.StaticFeedforward;
import com.rowanmcalpin.nextftc.ftc.hardware.controllables.HoldPosition;
import com.rowanmcalpin.nextftc.ftc.hardware.controllables.MotorEx;
import com.rowanmcalpin.nextftc.ftc.hardware.controllables.RunToPosition;

public class nextLift extends Subsystem {

    public static final nextLift INSTANCE = new nextLift();
    public nextLift() { }
    public MotorEx motor;
    String name = "EH";
    public PIDFController controller = new PIDFController(0.001, 0.02, 0.0, new StaticFeedforward(0.7));

//    public String EH = "EH";
    public Command setTolerance(int tolerance){
        return new LambdaCommand().setStart(()->{
            controller.setSetPointTolerance(tolerance);
        });
    }

    public Command toHeight(double height, double sec) {
        return new SequentialGroup(
                new RunToPosition(motor,height,controller,this),
                new HoldPosition(motor, controller, this).perpetually().endAfter(sec)

        );

    }


//    public Command cb() {
//       if(CommandManager.INSTANCE.hasCommands() == False)
//    }

    @Override
    public void initialize() {
        motor = new MotorEx(name);
        motor.resetEncoder();
        motor.reverse();

        setTolerance(100);

    }
}
