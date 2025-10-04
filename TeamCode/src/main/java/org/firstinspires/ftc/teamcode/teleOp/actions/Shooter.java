package org.firstinspires.ftc.teamcode.teleOp.actions;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Shooter {
    DcMotorEx shooter;
    Telemetry telemetry;
    public <roni2_intake> Shooter(DcMotorEx shooter, Telemetry telemetry){
        this.shooter = shooter;
        this.telemetry = telemetry;
    }
    public void shooterTest(double power){
        shooter.setPower(power);
    }
}
