package org.firstinspires.ftc.teamcode.Misc.Utils.filters;

import org.firstinspires.ftc.teamcode.Misc.Utils.AngleFunctions;

public class AngleLowPass extends Filter{
    LowPass lowPass;

    public double convertSignedDistToAngle(double dist){
        if(dist <= 0){
            return 180+dist;
        }
        else{
            return dist-180;
        }
    }
    @Override
    public void filter(double curr) {
        double signedDist = AngleFunctions.getDiffBetweenAngles(curr, 180);
        lowPass.update(signedDist);
        filtered = convertSignedDistToAngle(lowPass.get());
    }

    @Override
    @Deprecated
    public void start() {
    }

    public void start(double alpha) {
        lowPass = new LowPass();
        lowPass.start(alpha);
    }

    @Override
    public void update(double curr) {
        filter(curr);
    }

    @Override
    public void reset() {
        filtered = 0;
    }
}
