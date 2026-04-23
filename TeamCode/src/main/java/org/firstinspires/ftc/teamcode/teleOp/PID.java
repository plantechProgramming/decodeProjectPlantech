package org.firstinspires.ftc.teamcode.teleOp;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class PID {
    private static final ElapsedTime timer = new ElapsedTime();

    Utils utils = new Utils();

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

    int count = 0;
    double prevPower = 0;
    //TODO: make name fit all other funcs
    public double getPIDPowerWithT(double current){
        double pow;
        if(count % t == 0){
            double currentError = utils.getDiffBetweenAngles(wanted, current);
            pow = getPIDPower(currentError);
            telemetry.addLine("entered");
        }
        else{
            pow = prevPower;
        }
        prevPower = pow;
        count++;
        telemetry.addData("count", count);
        return pow;
    }

}
