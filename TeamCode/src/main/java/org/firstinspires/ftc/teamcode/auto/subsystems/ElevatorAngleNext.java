package org.firstinspires.ftc.teamcode.auto.subsystems;

//import com.dev.nextftc.core.Subsystem;
//import com.rowanmcalpin.nextftc.core.command.Command;
//import com.rowanmcalpin.nextftc.core.command.groups.ParallelRaceGroup;
//import com.rowanmcalpin.nextftc.core.command.groups.SequentialGroup;
//import com.rowanmcalpin.nextftc.core.command.utility.LambdaCommand;
//import com.rowanmcalpin.nextftc.core.command.utility.delays.Delay;
//import com.rowanmcalpin.nextftc.core.control.controllers.PIDFController;
//import com.rowanmcalpin.nextftc.core.control.controllers.feedforward.StaticFeedforward;
//import com.rowanmcalpin.nextftc.ftc.hardware.controllables.HoldPosition;
//import com.rowanmcalpin.nextftc.ftc.hardware.controllables.MotorEx;
//import com.rowanmcalpin.nextftc.ftc.hardware.controllables.RunToPosition;
//
//import org.jetbrains.annotations.NotNull;
//
//public class ElevatorAngleNext  extends Subsystem {
//    // BOILERPLATE
//    public static final ElevatorAngleNext INSTANCE = new ElevatorAngleNext();
//    private ElevatorAngleNext() { }
//    public MotorEx EA;
//
//    public PIDFController PID_EA = new PIDFController(0.004, 0, 0, new StaticFeedforward(0.1));
//
//    public Command setTolerance(int tolerance){
//        return new LambdaCommand().setStart(()->{
//            PID_EA.setSetPointTolerance(tolerance);
//        });
//    }
//    public Command toAngle(double angle, double sec) {
//
//        return new SequentialGroup(
//                new RunToPosition(EA,angle,PID_EA,this)
////                new ParallelRaceGroup(
////                        new HoldPosition(EA, PID_EA, this),
////                        new Delay(sec)
////                )
//        );
//
//    }
//
//
//    @NotNull
//    public Command getDefaultCommand() {
//        return new HoldPosition(EA,PID_EA, this);
//    }
//
//    @Override
//    public void initialize() {
//
//        EA = new MotorEx("EA");
//        EA.resetEncoder();
//        EA.reverse();
//        setTolerance(50);
//    }

//}
