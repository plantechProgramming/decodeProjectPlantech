package org.firstinspires.ftc.teamcode.Misc.Utils.filters;

import java.io.ObjectOutput;
import java.util.ArrayList;

public class AVG extends Filter<Double> {

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
    public void update(Double curr) {
        AVGsum += curr;
        AVGCounter++;
        filtered = AVGsum/AVGCounter;
    }
    @Override
    public void reset(){
        AVGsum = 0;
        AVGCounter = 0;
    }

}
