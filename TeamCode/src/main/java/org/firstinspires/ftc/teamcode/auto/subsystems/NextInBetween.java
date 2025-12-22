package org.firstinspires.ftc.teamcode.auto.subsystems;

import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.groups.ParallelGroup;
import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.hardware.impl.CRServoEx;
import dev.nextftc.hardware.powerable.SetPower;

public class NextInBetween implements Subsystem {
    public static final NextInBetween INSTANCE = new NextInBetween();
    public NextInBetween(){}
    CRServoEx ibl = new CRServoEx("IBL")
            ,ibr = new CRServoEx("IBR")
            ,sl = new CRServoEx("SIBL")
            ,sr = new CRServoEx("SIBR"); //ib = inbetween, s = shooter

    public Command inBetweenIn(){
        return new ParallelGroup(
                new SetPower(ibr, -1),
                new SetPower(ibl, 1),
                new SetPower(sr, -1),
                new SetPower(sl, 1)
        );
    }

public Command inBetweenOut() {
    return new ParallelGroup(
            new SetPower(ibr, 1),
            new SetPower(ibl, -1),
            new SetPower(sr, 1),
            new SetPower(sl, -1)
    );

}
}
