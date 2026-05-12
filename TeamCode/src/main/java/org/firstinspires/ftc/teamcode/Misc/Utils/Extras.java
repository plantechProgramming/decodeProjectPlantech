package org.firstinspires.ftc.teamcode.Misc.Utils;

import android.util.Pair;

public class Extras {
    public boolean threshold(double curr, double wanted, double threshold){
        return Math.abs(wanted - curr) < threshold;
    }
}
