package org.firstinspires.ftc.teamcode.Misc.Utils;

import android.util.Pair;

import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.Misc.InitMotors;
import org.firstinspires.ftc.teamcode.Misc.Utils.filters.LowPass;

import java.util.ArrayList;
import java.util.List;

public class Extras {
    public static final Pair<Double, Double> ROBOT_SIZE = new Pair<>(15.43, 17.32); // width, height in that order, in inches
    LowPass voltageLowPass = new LowPass(0.02);
    public double getVoltageCompensatedPow(double pow, double voltage){
        voltageLowPass.update(voltage);
        return pow * (14/voltageLowPass.get());
    }
    double jumps;
    ArrayList<Integer> histoList= new ArrayList<>();
    public void startHistogram(double jumps){
        this.jumps = jumps;
    }
    public void updateHistogram(double num){
        int currIndex = (int)(num/jumps)-1;
        for(int i=histoList.size(); i<=currIndex; i++) {
            histoList.add(0);
        }
        histoList.add(currIndex, histoList.get(currIndex)+1);
        histoList.remove(currIndex+1);
    }
    public ArrayList<Integer> getHistogram(){
        return histoList;
    }

}
