package org.firstinspires.ftc.teamcode.teleOp.actions;

import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.teleOp.PID;

public class Turret {
    public DcMotorEx turretMotor;
    private final double GEAR_RATIO = 1; // TODO: update for final version
    private final double TICK_PER_TURN = 38; // TODO: check!!
    public Turret(DcMotorEx turretMotor){
        this.turretMotor = turretMotor;
    }

    public void turnToDeg(double deg){
        double currentDeg = getCurDeg();
        PID pid = new PID(0.01, 0, 0, 0); // prev GOOD p = 0.022, i = 0.00000001, d = 0.000001, f = 0
        double threshold = 1;
        double power = 0;
        deg *= GEAR_RATIO;
        pid.setWanted(deg);

        if(Math.abs(deg - currentDeg) > threshold){ // if not in threshold
            power = pid.updatedeg(currentDeg);
            turretMotor.setPower(power);
        }
    }

    public double getCurDeg(){
        return tickToDeg(turretMotor.getCurrentPosition());
    }

    double tickToDeg(double ticks){
        double ticksPerDeg = TICK_PER_TURN/360;
        return ticks/ticksPerDeg;
    }
}
