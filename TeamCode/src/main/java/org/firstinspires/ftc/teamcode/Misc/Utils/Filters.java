package org.firstinspires.ftc.teamcode.Misc.Utils;

import java.util.ArrayList;
import java.util.Collections;

public class Filters {

    public boolean threshold(double curr, double wanted, double threshold){
        return Math.abs(wanted - curr) < threshold;
    }

    public double getAVGForList(ArrayList<Double> numbers){
        double sum = 0;
        for(double number : numbers){
            sum += number;
        }
        return sum/numbers.size();
    }

    double AVGsum = 0;
    double AVGCounter = 0;
    public void updateAVG(double curr){
        AVGsum += curr;
        AVGCounter++;
    }

    public double getAVG(){
        return AVGsum/AVGCounter;
    }
    public void resetAVG(){
        AVGsum = 0;
        AVGCounter = 0;
    }

    public double median(ArrayList<Double> numbers){
        Collections.sort(numbers);
        int size = numbers.size();
        if(size % 2 == 0){
            return (numbers.get(size/2) + numbers.get(size/2 - 1) / 2);
        }
        return numbers.get(size/2);
    }

    public ArrayList<Double> medianNumbers = new ArrayList<>();
    public void updateMedian(double val){
        medianNumbers.add(val);
    }



}
