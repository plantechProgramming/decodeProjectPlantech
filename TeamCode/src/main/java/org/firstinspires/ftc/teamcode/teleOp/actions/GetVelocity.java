package org.firstinspires.ftc.teamcode.teleOp.actions;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import dev.nextftc.hardware.impl.MotorEx;

public class GetVelocity {
    DcMotorEx motor;
    double alpha;
    int ticksPerRevolution = 8192; // 8192 for thru bore, 28 for built in

    public GetVelocity(DcMotorEx motor, double alpha) {
        this.motor = motor;
        this.alpha = alpha;
    }

    public GetVelocity(DcMotorEx motor, double alpha, final int ticksPerRevolution) {
        this.motor = motor;
        this.alpha = alpha;
        this.ticksPerRevolution = ticksPerRevolution;
    }

    private static final ElapsedTime timer = new ElapsedTime();
    public long prevEncoder = 0;
    public long curEncoder;
    public double prevTime = 0;
    public double curTime;

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
        double LOW_PASS_CONST = 1500;

        double timeDiff = curTime - prevTime;
        double encoderDiff = curEncoder - prevEncoder;

        double tickVelocity = encoderDiff / timeDiff; // ticks/milliseconds
        double curVelocity = (tickVelocity * millisecondsToMinute) / ticksPerRevolution;
        if (Math.abs(curVelocity - prevRawVelocity) > LOW_PASS_CONST) {
            curVelocity = prevRawVelocity;
        }
        prevRawVelocity = curVelocity;
        return curVelocity;
    }
    public double filtered(double alpha, double val, double prevVal){
        return alpha * val + (1 - alpha) * prevVal;
    }
}
