package org.firstinspires.ftc.teamcode.Misc;

import com.pedropathing.geometry.Pose;

public class DataSaving {
    private static Pose endPos = null;
    public static void setEndPos(Pose pose){
        endPos = pose;
    }

    public static Pose getEndPos() throws NullPointerException{
        if(endPos != null)
            return endPos;

        throw new NullPointerException("no position set from auto");
    }
}
