package org.firstinspires.ftc.teamcode.Misc;

import android.util.Pair;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

public class RobotPose { // everything is in CM

    GoBildaPinpointDriver odometry;
    Pose2D pose;
    Pair<Double, Double> posXY;

    public RobotPose(GoBildaPinpointDriver odometry){
        this.odometry = odometry;
    }

    public RobotPose(Pair<Double, Double> posXY){
        this.posXY = new Pair<>(posXY.first, posXY.second);
    }

    public RobotPose(Pose2D pose){
        this.pose = pose;
    }

    public double getX(){
        if(odometry != null){
            return odometry.getPosX(DistanceUnit.CM);
        }
        else if(pose != null){
            return pose.getX(DistanceUnit.CM);
        }
        else{
            return posXY.first;
        }
    }

    public double getY(){
        if(odometry != null){
            return odometry.getPosY(DistanceUnit.CM);
        }
        else if(pose != null){
            return pose.getY(DistanceUnit.CM);
        }
        else{
            return posXY.second;
        }
    }

    public double getHeading(){
        if(odometry != null){
            return odometry.getHeading(AngleUnit.DEGREES);
        }
        else if(pose != null){
            return pose.getHeading(AngleUnit.DEGREES);
        }
        else{
            throw new IllegalArgumentException("No heading given");
        }
    }
}
