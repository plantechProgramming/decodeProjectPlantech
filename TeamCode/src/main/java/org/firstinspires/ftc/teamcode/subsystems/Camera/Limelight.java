package org.firstinspires.ftc.teamcode.subsystems.Camera;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.Misc.InitMotors;
import org.firstinspires.ftc.teamcode.Misc.Utils.PoseFunctions;
import org.firstinspires.ftc.teamcode.Misc.Utils.filters.PoseLowPass;

public class Limelight {
    Limelight3A ll;
    PoseLowPass poseLowPass = new PoseLowPass(0.05, 0.03);
    public Limelight() {
        ll = InitMotors.ll;
    }

    public Pose3D getLatestBotpose() throws NullPointerException{ // LLCords
        LLResult result = ll.getLatestResult();
        if (result != null) {
            if (result.isValid()) {
                poseLowPass.update(PoseFunctions.pose3DToPose2D(result.getBotpose(), AngleUnit.DEGREES));
                return result.getBotpose();
            }
        }
        throw new NullPointerException("No valid apriltag found");
    }

    public Pose2D getFilteredBotPose() throws NullPointerException{
        getLatestBotpose();
        return poseLowPass.get();
    }
}
