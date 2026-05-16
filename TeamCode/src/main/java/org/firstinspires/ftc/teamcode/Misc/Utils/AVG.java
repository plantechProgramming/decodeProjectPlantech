package org.firstinspires.ftc.teamcode.Misc.Utils;

import java.util.ArrayList;

public class AVG extends Filter{

    public double getAVGForList(ArrayList<Double> numbers) {
        double sum = 0;
        for (double number : numbers) {
            sum += number;
        }
        return sum / numbers.size();
    }

    double AVGsum = 0;
    double AVGCounter = 0;
    @Override
    public void filter(double curr) {
        AVGsum += curr;
        AVGCounter++;
        filtered = AVGsum/AVGCounter;
    }

    @Override
    @Deprecated
    public void start() {}

    @Override
    public void update(double curr) {
        filter(curr);
    }

    @Override
    public void reset(){
        AVGsum = 0;
        AVGCounter = 0;
    }


}
