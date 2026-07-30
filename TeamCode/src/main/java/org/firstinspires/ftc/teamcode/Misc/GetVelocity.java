package org.firstinspires.ftc.teamcode.Misc;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Misc.Utils.filters.LowPass;

public class GetVelocity {
    DcMotorEx motor;
    int ticksPerRevolution = 8192;

    LowPass lowPass;
    public GetVelocity(DcMotorEx motor, double alpha) {
        this.motor = motor;
        lowPass = new LowPass(alpha);
    }
    public GetVelocity(DcMotorEx motor, double alpha, int ticksPerRevolution) {
        this.motor = motor;
        lowPass = new LowPass(alpha);
        this.ticksPerRevolution = ticksPerRevolution;
    }

    private static final ElapsedTime timer = new ElapsedTime();
    private long prevEncoder = 0;
    private double prevTime = 0;
    private double prevVelocity = 0;
    private final int MILLISECONDS_TO_MINUTE = 60000;

    public double getRawVelocity() {
        long curEncoder = motor.getCurrentPosition();
        double curTime = timer.milliseconds();

        double timeDiff = curTime - prevTime;
        long encoderDiff = curEncoder - prevEncoder;

        double tickVelocity = encoderDiff / timeDiff; // ticks/milliseconds
        double velocity = (tickVelocity * MILLISECONDS_TO_MINUTE) / ticksPerRevolution;

        prevTime = curTime;

        if(encoderDiff == 0){
            return prevVelocity;
        }

        prevEncoder = curEncoder;
        prevVelocity = velocity;
        System.out.println(velocity);
        lowPass.update(velocity);
        return velocity;
    }

    public double getVelocityFilter(){
        getRawVelocity();
        return lowPass.get();
    }
}
