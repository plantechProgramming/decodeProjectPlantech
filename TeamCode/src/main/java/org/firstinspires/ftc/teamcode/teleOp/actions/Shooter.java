package org.firstinspires.ftc.teamcode.teleOp.actions;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import dev.nextftc.core.units.Distance;

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
    public void shooterTest(double x){
        shooter2.setPower(x);
    }

    // g - gravity acceleration
    final double g = 9.8;
    // h - goal height + some 5 cm. IN CM
    final double h = 95;
    final double diameter = .096; //in mm
    final int MAX_RPM = 6000;
    public void noPhysShoot(double x){
        shooter2.setPower(-x);
        shooter.setPower(x);
    }
    public void shoot(Distance d, double t){
        double theta = Math.atan((g*t*t + 2*h)/(2*d.inCm));
        double velocity = d.inMm/(Math.cos(theta)*t);
        double rpm = (60/1)*velocity/(diameter*Math.PI*MAX_RPM);
        telemetry.addData("rpm", rpm);
        shooter.setPower(rpm);
        shooter2.setPower(-rpm);
    }

}
