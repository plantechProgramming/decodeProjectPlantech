package org.firstinspires.ftc.teamcode.auto.subsystems;

import com.acmerobotics.dashboard.FtcDashboard;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import dev.nextftc.core.commands.Command;
import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.hardware.impl.MotorEx;
import dev.nextftc.hardware.powerable.SetPower;

public class NextIntake implements Subsystem {
    public static final NextIntake INSTANCE = new NextIntake();
    public NextIntake() { }
    FtcDashboard dashboard = FtcDashboard.getInstance();
    Telemetry dashboardTelemetry =  dashboard.getTelemetry();
    MotorEx intakeMotor = new MotorEx("Intake");

    public Command take(){
        return new SetPower(intakeMotor,1);
    }
    public Command out(){
        return new SetPower(intakeMotor,-1);
    }
    public Command stop(){
        return new SetPower(intakeMotor,0);
    }
}
