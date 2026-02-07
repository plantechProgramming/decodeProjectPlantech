package org.firstinspires.ftc.teamcode.teleOp;

import com.qualcomm.robotcore.util.ElapsedTime;

public class PID {
    private static final ElapsedTime timer = new ElapsedTime();

    public double kP = 0;
    public double kI = 0;
    public double kD = 0;
    public double kF = 0;
    public double iZone = 0;

    public double wanted = 0;

    private double integral = 0;
    private double power = 0;

    private double prevError = 0;
    private double prevTime = 0;

    public PID(final double kP, final double kI, final double kD, final double kF) {
        this.kP = kP;
        this.kI = kI;
        this.kD = kD;
        this.kF = kF;
//        this.iZone = iZone;
    }

    public void setWanted(final double wanted) {this.wanted = wanted;}

    public double update(final double current) {
        final double currentError = wanted - current;
        final double currentTime = timer.milliseconds();
        final double deltaTime = currentTime - prevTime;

        return getPIDPower(currentError);
    }
    public double updatedeg(final double current) {
        double currentError = wanted - current;
        if (currentError < -180){
            currentError += 360;
        }
        if (currentError > 180){
            currentError -= 360;
        }
        return getPIDPower(currentError);
    }

//    public double updateTurretDeg(final double current){
//
//    }

    public double getPIDPower(final double currentError){
        double currentTime = timer.milliseconds();
        double deltaTime = currentTime - prevTime;

        integral += currentError * deltaTime;
        final double derivative = deltaTime == 0 ? 0 : (currentError - prevError) / deltaTime;

        prevError = currentError;
        prevTime = currentTime;
        power = kP * currentError + kI * integral + kD * derivative + kF * wanted;
        return power;
    }
}