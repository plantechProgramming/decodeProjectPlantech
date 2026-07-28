package org.firstinspires.ftc.teamcode.Misc.Utils.filters;

public class LowPass extends Filter<Double> {
    double alpha;
    public LowPass(double alpha) {
        this.alpha = alpha;
    }


    @Override
    public void update(Double curr) {
        filtered = alpha * curr + (1 - alpha) * filtered;
    }

    @Override
    public void reset(){
        filtered = 0.0;
    }

}
