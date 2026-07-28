package org.firstinspires.ftc.teamcode.Misc.Utils.filters;

import org.firstinspires.ftc.teamcode.Misc.Utils.AngleFunctions;

public class AngleLowPass extends Filter<Double>{
    LowPass lowPass;
    public AngleLowPass(double alpha){
        lowPass = new LowPass(alpha);
    }

    private double convertSignedDistToAngle(double dist){
        if(dist <= 0){
            return 180+dist;
        }
        else{
            return dist-180;
        }
    }

    @Override
    public void update(Double curr) { // note that there is a problem when the angle is near zero
        double signedDist = AngleFunctions.getDiffBetweenAngles(curr, 180);
        lowPass.update(signedDist);
        filtered = convertSignedDistToAngle(lowPass.get());
    }

    @Override
    public void reset() {
        filtered = 0.0;
    }
}
