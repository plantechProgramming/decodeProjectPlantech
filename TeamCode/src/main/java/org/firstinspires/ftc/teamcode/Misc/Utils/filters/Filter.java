package org.firstinspires.ftc.teamcode.Misc.Utils.filters;

public abstract class Filter {
    double filtered;
    public double get(){
        return filtered;
    }
    public abstract void filter(double curr);
    public abstract void start();
    public abstract void update(double curr);
    public abstract void reset();
}
