package org.firstinspires.ftc.teamcode.subsystems.Camera;

import android.util.Pair;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.Misc.InitMotors;
import org.firstinspires.ftc.teamcode.Misc.Utils.PoseFunctions;
import org.firstinspires.ftc.teamcode.Misc.Utils.filters.PoseLowPass;

import java.util.ArrayList;
import java.util.List;

public class Limelight {
    Limelight3A ll;
    PoseLowPass poseLowPass = new PoseLowPass(0.01, 0.01);
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

    public List<LLResultTypes.ColorResult>  getLatestColorDetection() throws NullPointerException{ // LLCords
        LLResult result = ll.getLatestResult();
        if (result != null && result.isValid()) {
            return result.getColorResults();
        }
        throw new NullPointerException("No valid color detection Found found");
    }

    public Pose2D getFilteredBotPose() throws NullPointerException{
        getLatestBotpose();
        return poseLowPass.get();
    }

    public List<Pair<Double, Double>> getPose2DFromColorDetection() throws NullPointerException{
        double llHeight = 39; // cm
        double artifactHeight = 8; // approximately in cm
        List<Pair<Double, Double>> XYlist = new ArrayList<>();
        List<LLResultTypes.ColorResult> colorDetections = getLatestColorDetection();
        for(LLResultTypes.ColorResult colorDetection : colorDetections){
            double xRad = Math.toRadians(colorDetection.getTargetXDegrees());
            double yRad = Math.toRadians(colorDetection.getTargetYDegrees());
            double heightDiff = llHeight - artifactHeight;

            double groundDis = heightDiff / Math.tan(yRad); // from cam to target
            double actualY = Math.cos(xRad) * groundDis;
            double actualX = Math.sin(xRad) * groundDis;
            XYlist.add(new Pair<>(actualX, actualY));
        }
        return XYlist;
    }
}
