package org.firstinspires.ftc.teamcode.subsystems.Camera;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.Misc.Utils.filters.AngleLowPass;

public class Limelight {
    Limelight3A ll;

    AngleLowPass angleLowPass = new AngleLowPass();
    public Limelight(Limelight3A ll) {
        this.ll = ll;
        angleLowPass.start(0.03);
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

    public double getRawHeadingLLCords() throws NullPointerException{
        Pose3D botPose = getLatestBotpose();
        return botPose.getOrientation().getYaw();
    }

    public double getFilteredHeadingLLCoords() throws NullPointerException{
        return angleLowPass.get();
    }

    public void updateFilter() throws NullPointerException{
        double heading = getRawHeadingLLCords();
        angleLowPass.update(heading);
    }
}
