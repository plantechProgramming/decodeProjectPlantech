package org.firstinspires.ftc.teamcode.auto.subsystems;

import dev.nextftc.control.ControlSystem;
import dev.nextftc.core.commands.Command;
import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.hardware.controllable.RunToPosition;
import dev.nextftc.hardware.impl.MotorEx;

public class ElevatorAngleNext implements Subsystem {
    public static final ElevatorAngleNext INSTANCE = new ElevatorAngleNext();

    private ElevatorAngleNext() {
    }

    private MotorEx motor = new MotorEx("EA")
            .reversed()
            .zeroed()
            .brakeMode();

    private ControlSystem controlSystem = ControlSystem.builder()
            .posPid(0.005, 0, 0)
            .elevatorFF(0)
            .build();

    public Command toAngle(int height,int tolerance){
        return new RunToPosition(controlSystem,height,tolerance).requires(this);
    }
    @Override
    public void periodic() {
        motor.setPower(controlSystem.calculate(motor.getState()));
    }
}
