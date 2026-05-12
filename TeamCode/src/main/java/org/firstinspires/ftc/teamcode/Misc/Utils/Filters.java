package org.firstinspires.ftc.teamcode.Misc.Utils;

import java.util.ArrayList;
import java.util.Collections;

public class Filters {
    public class AVG{
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
    }
    public class Median{
        private double median(ArrayList<Double> numbers){
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

        public double getMedian(){
            return median(medianNumbers);
        }

        public void resetMedian(){
            medianNumbers.clear();
        }
    }

    public class LowPass{
        double prevVal = 0;
        double alpha = 0.1;
        private void filter(double val){
            prevVal = alpha * val + (1 - alpha) * prevVal;
        }

        public void startFilter(double alpha){
            this.alpha = alpha;
        }

        public void updateFilter(double val){
            filter(val);
        }

        public double getFilter(){
            return prevVal;
        }

        public void resetFilter(){
            prevVal = 0;
        }

        public void resetAlpha(){
            alpha = 0;
        }
    }
}
