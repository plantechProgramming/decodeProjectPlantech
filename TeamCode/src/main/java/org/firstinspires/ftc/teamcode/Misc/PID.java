package org.firstinspires.ftc.teamcode.Misc;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Misc.Utils.AngleFunctions;

public class PID {
    private static final ElapsedTime timer = new ElapsedTime();
    AngleFunctions angleFunctions = new AngleFunctions();
    public double kP = 0;
    public double kI = 0;
    public double kD = 0;
    public double kF = 0;
    public double iZone = 0;
    public Telemetry telemetry;
    public double wanted = 0;
    private double integral = 0;
    private double power = 0;
    private double prevError = 0;
    private double prevTime = 0;
    private int t = 0;

    public PID(final double kP, final double kI, final double kD, final double kF) {
        this.kP = kP;
        this.kI = kI;
        this.kD = kD;
        this.kF = kF;
//        this.iZone = iZone;
    }

    // t = amt of loops between checks
    public PID(final double kP, final double kI, final double kD, final double kF, final int t, Telemetry telemetry){
        this.kP = kP;
        this.kI = kI;
        this.kD = kD;
        this.kF = kF;
        this.t = t;
        this.telemetry = telemetry;
    }

    public void setWanted(final double wanted) {this.wanted = wanted;}

    public double update(final double current) {
        return getPIDPower(wanted - current);
    }
    public double updatedeg(final double current) {
        double currentError = wanted - current;
        angleFunctions.convertToWrapAroundAngle(currentError);
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
//        telemetry.addData("derivative",derivative);
//        telemetry.addData("integral", integral);
//        telemetry.addData("error", currentError);
        return power;
    }

}
