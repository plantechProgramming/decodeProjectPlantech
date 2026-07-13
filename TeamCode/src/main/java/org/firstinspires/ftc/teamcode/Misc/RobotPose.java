package org.firstinspires.ftc.teamcode.Misc;

import android.util.Pair;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

public class RobotPose { // everything is in CM

    GoBildaPinpointDriver odometry;

    public RobotPose(GoBildaPinpointDriver odometry){
        this.odometry = odometry;
    }

    public double getX(){
        if(odometry != null){
            return odometry.getPosX(DistanceUnit.CM);
        }
        else{
            throw new IllegalArgumentException("No X pos given");
        }
    }

    public double getY(){
        if(odometry != null){
            return odometry.getPosY(DistanceUnit.CM);
        }
        else{
            throw new IllegalArgumentException("No Y pos given");
        }
    }

    public double getHeading(){
        if(odometry != null){
            return odometry.getHeading(AngleUnit.DEGREES);
        }
        else{
            throw new IllegalArgumentException("No heading given");
        }
    }
}
