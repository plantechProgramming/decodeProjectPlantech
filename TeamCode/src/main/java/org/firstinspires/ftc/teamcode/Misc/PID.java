package org.firstinspires.ftc.teamcode.Misc;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Misc.Utils.AngleFunctions;

public class PID {
    private static final ElapsedTime timer = new ElapsedTime();
    private double kP = 0;
    private double kI = 0;
    private double kD = 0;
    private double kF = 0;
    private double iZone = 0;
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
        return getPIDPower(wanted - current);
    }
    public double updateDeg(final double current) {
        double currentError = wanted - current;
        currentError = AngleFunctions.convertToWrapAroundAngle(currentError);
        return getPIDPower(currentError);
    }

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
