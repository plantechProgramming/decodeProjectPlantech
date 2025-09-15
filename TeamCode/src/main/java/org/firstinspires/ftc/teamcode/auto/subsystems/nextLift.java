package org.firstinspires.ftc.teamcode.auto.subsystems;

import com.qualcomm.robotcore.hardware.DcMotorEx;

import dev.nextftc.control.ControlSystem;
import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.groups.SequentialGroup;
import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.hardware.controllable.Controllable;
import dev.nextftc.hardware.controllable.RunToPosition;
import dev.nextftc.hardware.impl.MotorEx;

public class nextLift implements Subsystem {
    public static final nextLift INSTANCE = new nextLift();
    private nextLift() { }

    private MotorEx motor = new MotorEx("EH")
            .zeroed()
            .reversed()
            .brakeMode();

    private ControlSystem controlSystem = ControlSystem.builder()
            .posPid(0.005, 0, 0)
            .elevatorFF(0)
            .build();

    public Command toHeight(int height,int tolerance){
        return new RunToPosition(controlSystem,height,tolerance).requires(this);
    }


    @Override
    public void periodic() {
        motor.setPower(controlSystem.calculate(motor.getState()));

    }
}
