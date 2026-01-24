package org.firstinspires.ftc.teamcode.auto.autos;

import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.util.ReadWriteFile;

import org.firstinspires.ftc.robotcore.internal.system.AppUtil;

import java.io.File;

public class ReadWrite {
    File file = AppUtil.getInstance().getSettingsFile("lastLoc.txt");
    public void writePose(Pose pose){
        String poseString = pose.toString();
        poseString = poseString.substring(1, poseString.length() - 1);
        ReadWriteFile.writeFile(file, poseString); // replaces the current text
    }

    public Pose readPose(){
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
