package org.firstinspires.ftc.teamcode.auto.subsystems;

import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.groups.ParallelGroup;
import dev.nextftc.core.commands.groups.SequentialGroup;
import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.core.units.Distance;
import dev.nextftc.hardware.impl.MotorEx;
import dev.nextftc.hardware.impl.ServoEx;
import dev.nextftc.hardware.positionable.SetPosition;
import dev.nextftc.hardware.powerable.SetPower;

public class NextShooter implements Subsystem {
    public static final NextShooter INSTANCE = new NextShooter();
    public NextShooter() { }
    private MotorEx shooter1 = new MotorEx("shooter");
    private MotorEx shooter2 = new MotorEx("shooter2");
    private ServoEx hood = new ServoEx("hood");

    // g - gravity acceleration
    final double g = 9.8;
    // h - goal height + some 5 cm. IN CM
    final double h = 95;
    final double diameter = .096; //in mm
    final int MAX_RPM = 6000;
    public Command shoot(Distance d, double t){
        double theta = Math.atan((g*t*t + 2*h)/(2*d.inCm));
        double velocity = d.inCm/(Math.cos(theta)*t);
        double rpm = velocity/(diameter*Math.PI*MAX_RPM);
        return new SequentialGroup(
                new SetPosition(hood,theta),
                new ParallelGroup(
                        new SetPower(shooter1, rpm),
                        new SetPower(shooter2, -rpm)
                )
        );
    }

}
