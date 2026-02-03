package org.firstinspires.ftc.teamcode.teleOp.actions;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.teleOp.PID;

public class Turret {
    public DcMotorEx turretMotor;
    private final double GEAR_RATIO = 2.5; // TODO: update for final version
    private final double TICK_PER_TURN; // TODO: check!!
    private final double TICK_PER_DEG;
    public Turret(DcMotorEx turretMotor){
        this.turretMotor = turretMotor;
        TICK_PER_TURN = 1425.2;
        TICK_PER_DEG = TICK_PER_TURN/360;
    }

//    public void turnToDeg(double deg){
//        double currentDeg = getCurDeg();
//        PID pid = new PID(0.01, 0, 0, 0);
//        double threshold = 1;
//        double power = 0;
//        deg *= GEAR_RATIO;
//        pid.setWanted(deg);
//
//        if(Math.abs(deg - currentDeg) > threshold){ // if not in threshold
//            power = pid.updatedeg(currentDeg);
//            turretMotor.setPower(0.5);
//        }
//    }

    public double getCurDeg(){
        double deg = turretMotor.getCurrentPosition()/TICK_PER_DEG;
        return deg/GEAR_RATIO;
    }
    public double getRev(){
        return turretMotor.getCurrentPosition()/TICK_PER_TURN * GEAR_RATIO;
    }

    public int degToTicks(double deg){
        double ticks = deg * TICK_PER_DEG;
        return (int)Math.round(ticks*GEAR_RATIO); // FUCK JAVA
    }
    public void  turret(double deg){
        turretMotor.setTargetPosition(degToTicks(deg));
        turretMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        turretMotor.setVelocity(turretMotor.getMotorType().getAchieveableMaxTicksPerSecond());
    }
}
