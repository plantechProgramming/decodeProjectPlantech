package org.firstinspires.ftc.teamcode.Misc.Utils;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import java.util.ArrayList;
import java.util.Collections;

public class Median extends Filter{
    private ArrayList<Double> medianNumbers = new ArrayList<>();
    private void filter(ArrayList<Double> numbers){
        Collections.sort(numbers);
        int size = numbers.size();
        if(size % 2 == 0){
            filtered = (numbers.get(size/2) + numbers.get(size/2 - 1) / 2);
        }
        filtered = numbers.get(size/2);
    }


    @Override
    @Deprecated
    public void filter(double curr) {}

    @Override
    @Deprecated
    public void start() {}

    @Override
    public void update(double curr) {
        medianNumbers.add(curr);
        filter(medianNumbers);
    }

    @Override
    public void reset() {
        medianNumbers.clear();
    }
}
