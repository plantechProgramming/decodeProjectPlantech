package org.firstinspires.ftc.teamcode.Misc.Utils;

import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.Misc.InitMotors;
import org.firstinspires.ftc.teamcode.Misc.Utils.filters.LowPass;

import java.util.ArrayList;
import java.util.List;

public class Extras {
    LowPass lowPass = new LowPass();
    public double getVoltageCompensatedPow(double pow, double voltage){
        lowPass.start(0.02);
        lowPass.update(voltage);
        return pow * (14/lowPass.get());
    }
    double[] histoList;
    double jumps;
    public void startHistogram(int lenHisto, double jumps){
        histoList = new double[lenHisto];
        this.jumps = jumps;
    }
    public void updateHistogram(double num){
        histoList[(int)(num/jumps)]++;
    }
    public double[] getHistogram(){
        return histoList;
    }

}
