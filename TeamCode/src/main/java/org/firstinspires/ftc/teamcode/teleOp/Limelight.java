package org.firstinspires.ftc.teamcode.teleOp;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;

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

    public double getRawHeadingOdoCoords() throws NullPointerException{
        Pose3D botPose = getLatestBotpose();
        double heading = botPose.getOrientation().getYaw();
        return covertLLHeadingToOdo(heading);
    }

    // WORK IN PROGRESS, please dont kill this, i had like 15 min. ill get it done
    // write some pseudo in lessons and stuff
    double prevHeading = 0;
    public double getFilteredHeadingOdoCoords(double alpha) throws NullPointerException{
        double heading = getRawHeadingOdoCoords();
        double filteredHeading = utils.updateWraparoundFilter(alpha, heading, prevHeading);
        prevHeading = heading;
        return filteredHeading;
    }

    public void updateFilter(double alpha) throws NullPointerException{
        double heading = getRawHeadingOdoCoords();
        utils.updateWraparoundFilter(alpha, heading, prevHeading);
        prevHeading = heading;
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
