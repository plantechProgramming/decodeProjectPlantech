package org.firstinspires.ftc.teamcode.Misc;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Misc.Utils.Filters;

public class GetVelocity {
    Filters filters = new Filters();
    Filters.LowPass lowPass = filters.new LowPass();
    DcMotorEx motor;
    double alpha;
    int ticksPerRevolution = 28;
    public GetVelocity(DcMotorEx motor, double alpha) {
        this.motor = motor;
        this.alpha = alpha;
    }
    public GetVelocity(DcMotorEx motor, double alpha, int ticksPerRevolution) {
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

    public double getVelocityFilter() {
        lowPass.startFilter(alpha);
        curEncoder = motor.getCurrentPosition();
        curTime = timer.milliseconds();

        double timeDiff = curTime - prevTime;
        double encoderDiff = curEncoder - prevEncoder;

        double tickVelocity = encoderDiff / timeDiff; // ticks/milliseconds
        double velocity = (tickVelocity * millisecondsToMinute) / ticksPerRevolution;

        prevTime = curTime;
        prevEncoder = curEncoder;
        lowPass.updateFilter(velocity);
        return lowPass.getFilter();
    }
}
