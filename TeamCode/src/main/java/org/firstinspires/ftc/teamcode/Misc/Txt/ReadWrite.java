package org.firstinspires.ftc.teamcode.Misc.Txt;

import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.util.ReadWriteFile;

import org.firstinspires.ftc.robotcore.internal.system.AppUtil;

import java.io.File;

public class ReadWrite {
    static File file = AppUtil.getInstance().getSettingsFile("lastLoc.txt");
    public static void writePose(Pose pose){
        String poseString = pose.toString();
        poseString = poseString.substring(1, poseString.length() - 1);
        ReadWriteFile.writeFile(file, poseString); // replaces the current text
    }

    public static Pose readPose(){
        String[] posePartitions = ReadWriteFile.readFile(file).trim().split(", ");
        return new Pose(
                Double.parseDouble(posePartitions[0]),
                Double.parseDouble(posePartitions[1]),
                Math.toRadians(Double.parseDouble(posePartitions[2]))
        );
    }

//    public int getFileLength(){
//        String[] posePartitions = ReadWriteFile.readFile(file).trim().split(", ");
//        return posePartitions.length;
//    }

//    public void clear(){
//        ReadWriteFile.
//    }
}
