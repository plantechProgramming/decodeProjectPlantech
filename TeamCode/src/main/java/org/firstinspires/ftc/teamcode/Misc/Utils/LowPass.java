package org.firstinspires.ftc.teamcode.Misc.Utils;

public class LowPass extends Filter{
    double prevVal = 0;
    double alpha = 0.1;
    @Override
    public void filter(double curr){
        prevVal = alpha * val + (1 - alpha) * prevVal;
    }

    @Override
    public void start(double alpha){
        this.alpha = alpha;
    }

    @Override
    public void update(double val){
        filter(val);
    }

    @Override
    public void reset(){
        prevVal = 0;
    }

    public void resetAlpha(){
        alpha = 0;
    }
}
