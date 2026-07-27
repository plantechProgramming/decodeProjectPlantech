package org.firstinspires.ftc.teamcode.subsystems.Camera;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.Misc.InitMotors;
import org.firstinspires.ftc.teamcode.Misc.Utils.filters.AngleLowPass;

public class Limelight {
    Limelight3A ll;

    AngleLowPass angleLowPass = new AngleLowPass();
    public Limelight() {
        ll = InitMotors.ll;
        angleLowPass.start(0.03);
    }

    public Pose3D getLatestBotpose() throws NullPointerException{ // LLCords
        LLResult result = ll.getLatestResult();
        if (result != null) {
            if (result.isValid()) {
                return result.getBotpose();
            }
        }
        throw new NullPointerException("No valid apriltag found");
    }

    public double getRawHeading() throws NullPointerException{ // LLCords
        Pose3D botPose = getLatestBotpose();
        return botPose.getOrientation().getYaw();
    }

    public double getFilteredHeading() throws NullPointerException{ // LLCords
        return angleLowPass.get();
    }

    public void updateHeadingFilter() throws NullPointerException{ // LLCords
        double heading = getRawHeading();
        angleLowPass.update(heading);
    }
}
