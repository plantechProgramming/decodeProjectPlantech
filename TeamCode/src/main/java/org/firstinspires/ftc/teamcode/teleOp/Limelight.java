package org.firstinspires.ftc.teamcode.teleOp;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;

public class Limelight {
    Limelight3A ll;
    public Limelight(Limelight3A ll){
        this.ll = ll;
    }
    public void start(){
        ll.start();
    }
    public void stop(){
        ll.stop();
    }
    public void shutDown(){
        ll.shutdown();
    }

    public Pose3D getLatestBotpose(){
        LLResult result = ll.getLatestResult();
        if (result != null) {
            if (result.isValid()) {
                return result.getBotpose();
            }
        }
        return null;
    }


    public double covertLLHeadingToOdo(double heading){
        if(heading > 0 && heading < 90) {
            return heading + 90;
        }
        else {
            return heading - 270;
        }
    }
}
