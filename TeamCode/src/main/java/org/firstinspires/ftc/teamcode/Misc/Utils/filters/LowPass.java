package org.firstinspires.ftc.teamcode.Misc.Utils.filters;

public class LowPass extends Filter {
    double alpha = 0.1;
    @Override
    public void filter(double curr){
        filtered = alpha * curr + (1 - alpha) * filtered;
    }

    @Override
    @Deprecated
    public void start() {

    }
    public void start(double alpha){
        this.alpha = alpha;
    }

    @Override
    public void update(double val){
        filter(val);
    }

    @Override
    public void reset(){
        filtered = 0;
    }

}
