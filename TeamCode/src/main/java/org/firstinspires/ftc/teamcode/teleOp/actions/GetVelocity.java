package org.firstinspires.ftc.teamcode.teleOp.actions;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;
public class GetVelocity {
    DcMotorEx motor;
    double alpha;

    public GetVelocity(DcMotorEx motor, double alpha) {
        this.motor = motor;
        this.alpha = alpha;
    }


    private static final ElapsedTime timer = new ElapsedTime();
    public long prevEncoder = 0;
    public long curEncoder;
    public double prevTime = 0;
    public double curTime;
    int ticksPerRevolution = 8192; // for thru bore, 28 for built in
    int millisecondsToMinute = 60000;
    double prevVelocity = 0;

    public double getVelocityFilter() {
        double velocity = getRawVelocity();

        prevTime = curTime;
        prevEncoder = curEncoder;
        double filteredVelocity = filtered(alpha, velocity, prevVelocity);
        prevVelocity = filteredVelocity;
        return filteredVelocity;
    }

    double prevRawVelocity;
    public double getRawVelocity() {
        curEncoder = motor.getCurrentPosition();
        curTime = timer.milliseconds();
        double LOW_PASS_CONST = 500;

        double timeDiff = curTime - prevTime;
        double encoderDiff = curEncoder - prevEncoder;

        double tickVelocity = encoderDiff / timeDiff; // ticks/milliseconds
        double curVelocity = (tickVelocity * millisecondsToMinute) / ticksPerRevolution;
        if(Math.abs(prevRawVelocity - curVelocity) > LOW_PASS_CONST){
            curVelocity = prevRawVelocity;
        }
        prevRawVelocity = curVelocity;
        return curVelocity;
    }
    public double filtered(double alpha, double val, double prevVal){
        return alpha * val + (1 - alpha) * prevVal;
    }
}
