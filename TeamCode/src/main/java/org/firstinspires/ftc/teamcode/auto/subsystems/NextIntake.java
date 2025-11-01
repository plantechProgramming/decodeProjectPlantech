package org.firstinspires.ftc.teamcode.auto.subsystems;

import dev.nextftc.core.commands.Command;
import dev.nextftc.hardware.impl.MotorEx;
import dev.nextftc.hardware.powerable.SetPower;

public class NextIntake {
    public static final NextShooter INSTANCE = new NextShooter();
    public NextIntake() { }
    MotorEx motor = new MotorEx("intake");

    public Command take(){
        return new SetPower(motor,1);
    }
}
