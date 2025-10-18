package org.firstinspires.ftc.teamcode.teleOp.actions;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@Configurable
public class Shooter {
    DcMotorEx shooter, shooter2;
    Telemetry telemetry;
    public <roni2_intake> Shooter(DcMotorEx shootMotor, Telemetry telemetry, DcMotorEx shooter2){
        this.shooter = shootMotor;
        this.telemetry = telemetry;
        this.shooter2 = shooter2;
    }

    public static double curPower = 0;
    public void shooterTest(boolean x){
        if(!x){return;}
        shooter2.setPower(curPower);
    }
}
