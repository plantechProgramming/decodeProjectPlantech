package org.firstinspires.ftc.teamcode.Misc.Utils.filters;

public abstract class Filter<T> {
    T filtered;
    public T get(){
        return filtered;
    }
    public abstract void update(T curr);
    public abstract void reset();
}
