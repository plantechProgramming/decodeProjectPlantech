package org.firstinspires.ftc.teamcode.Misc;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Misc.Utils.filters.LowPass;

public class GetVelocity {
    DcMotorEx motor;
    double alpha;
    int ticksPerRevolution = 8192;

    LowPass lowPass = new LowPass();
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
    private long prevEncoder = 0;
    private double prevTime = 0;
    private final int MILLISECONDS_TO_MINUTE = 60000;

    public double getRawVelocity() {
        lowPass.start(alpha);
        long curEncoder = motor.getCurrentPosition();
        double curTime = timer.milliseconds();

        double timeDiff = curTime - prevTime;
        double encoderDiff = curEncoder - prevEncoder;

        double tickVelocity = encoderDiff / timeDiff; // ticks/milliseconds
        double velocity = (tickVelocity * MILLISECONDS_TO_MINUTE) / ticksPerRevolution;

        prevTime = curTime;
        prevEncoder = curEncoder;
        lowPass.update(velocity);
        return velocity;
    }

    public double getVelocityFilter(){
        return lowPass.get();
    }
}
