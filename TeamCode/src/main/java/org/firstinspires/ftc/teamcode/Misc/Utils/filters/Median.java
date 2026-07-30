package org.firstinspires.ftc.teamcode.Misc.Utils.filters;

import java.util.ArrayList;
import java.util.Collections;

public class Median extends Filter<Double> {
    public Median(){
        filtered = 0.0;
    }
    private ArrayList<Double> medianNumbers = new ArrayList<>();

    @Override
    public void update(Double curr) {
        medianNumbers.add(curr);
        Collections.sort(medianNumbers);
        int size = medianNumbers.size();
        if(size % 2 == 0){
            filtered = (medianNumbers.get(size/2) + medianNumbers.get(size/2 - 1) / 2);
        }
        filtered = medianNumbers.get(size/2);
    }

    @Override
    public void reset() {
        medianNumbers.clear();
    }
}
