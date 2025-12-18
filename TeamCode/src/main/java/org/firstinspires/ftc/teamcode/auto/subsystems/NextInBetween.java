package org.firstinspires.ftc.teamcode.auto.subsystems;

import com.qualcomm.robotcore.hardware.CRServo;

import dev.nextftc.core.commands.groups.ParallelGroup;
import dev.nextftc.hardware.impl.CRServoEx;
import dev.nextftc.core.commands.Command;
import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.hardware.powerable.SetPower;

public class NextInBetween implements Subsystem {
    public static final NextInBetween INSTANCE = new NextInBetween();
    CRServoEx ibl = new CRServoEx("IBL")
            ,ibr = new CRServoEx("IBR")
            ,sl = new CRServoEx("SIBL")
            ,sr = new CRServoEx("SIBR"); //ib = inbetween, s = shooter

    private NextInBetween() { }
    public Command inBetweenFunc(boolean in, boolean out){
        if(in){
            return new ParallelGroup(
                new SetPower(ibr, 1),
                new SetPower(ibl, -1),
                new SetPower(sr, 1),
                new SetPower(sl, -1)
            );


        }
        else if(out){
            return new ParallelGroup(
                    new SetPower(ibr, -1),
                    new SetPower(ibl, 1),
                    new SetPower(sr, -1),
                    new SetPower(sl, 1)
            );
        }
        else{
            return new ParallelGroup(
                    new SetPower(ibr, 0),
                    new SetPower(ibl, 0),
                    new SetPower(sr, 0),
                    new SetPower(sl, 0)
            );
        }
    }
    public Command inBetween2Shooter(boolean up, boolean down) {
        if (up) {
            return new ParallelGroup(
                    new SetPower(sr, 1),
                    new SetPower(sl, -1)
            );
        } else if (down) {
            return new ParallelGroup(
                    new SetPower(sr, -1),
                    new SetPower(sl, 1)
            );
        }else {
            return new ParallelGroup(
                    new SetPower(sr, 0),
                    new SetPower(sl, 0)
            );
        }
    }
}
