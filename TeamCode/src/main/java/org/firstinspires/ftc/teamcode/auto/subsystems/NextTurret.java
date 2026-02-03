package org.firstinspires.ftc.teamcode.auto.subsystems;

import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.hardware.impl.MotorEx;

public class NextTurret implements Subsystem {
    private MotorEx turretMotor;
    private final double GEAR_RATIO = 2.5;
    private final double TICK_PER_TURN = 1425.2;
}
