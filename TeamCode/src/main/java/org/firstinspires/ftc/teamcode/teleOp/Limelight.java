package org.firstinspires.ftc.teamcode.teleOp;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;

public class Limelight {
    Limelight3A ll;
    Utils utils = new Utils();
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

    public Pose3D getLatestBotpose() throws NullPointerException{
        LLResult result = ll.getLatestResult();
        if (result != null) {
            if (result.isValid()) {
                return result.getBotpose();
            }
        }
        throw new NullPointerException("No valid apriltag found");
    }

    public double getRawHeadingLLCoords() throws NullPointerException{
        Pose3D botPose = getLatestBotpose();
        return botPose.getOrientation().getYaw();
    }

    public double getFilteredHeadingOdoCoords() throws NullPointerException{
        return convertLLHeadingToOdo(getFilteredHeadingLLCoords());
    }

    public double getFilteredHeadingLLCoords() throws NullPointerException{
        return utils.getWraparoundFilter();
    }

    public void updateFilter() throws NullPointerException{
        double heading = getRawHeadingLLCoords();
        utils.updateWraparoundFilter(heading);
    }
    public double getRawHeadingOdoCoords() throws NullPointerException{
        return convertLLHeadingToOdo(getRawHeadingLLCoords());
    }
    public double convertLLHeadingToOdo(double heading){
        if(heading > 0 && heading < 90) {
            return heading + 90;
        }
        else {
            return heading - 270;
        }
    }
}
