package org.firstinspires.ftc.teamcode.teleOp;

import android.util.Pair;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.opencv.core.Mat;
import org.opencv.core.Point;

public class Utils {

    GoBildaPinpointDriver odometry;
    Telemetry telemetry;
    public Utils(Telemetry telemetry, GoBildaPinpointDriver odometry){
        this.telemetry = telemetry;
        this.odometry = odometry;
    }
    public final double LEN_FIELD = 360.172; // in cm
    double yOffset = 20;//prev = 18
    double xOffset = 20; // prev = 16
    public final Pose2D GOAL_RED = new Pose2D(DistanceUnit.CM,-LEN_FIELD/2-xOffset, -LEN_FIELD/2 - yOffset,AngleUnit.DEGREES,0);
    public final Pose2D GOAL_BLUE = new Pose2D(DistanceUnit.CM,LEN_FIELD/2-xOffset, -LEN_FIELD/2 + yOffset,AngleUnit.DEGREES,0);

    // fuck this, its hoory's fault
    public double square(double num){
        return Math.pow(num, 2);
    }
    public Pair<Double, Double> getXYdistToPoint(Pose2D point){
        double robotX = odometry.getPosX(DistanceUnit.CM);
        double robotY = odometry.getPosY(DistanceUnit.CM);
        double pointX = point.getX(DistanceUnit.CM);
        double pointY = point.getY(DistanceUnit.CM);
        return new Pair<>(pointX-robotX, pointY-robotY);
    }

    public Pair<Double,Double> getXYdistToGoal(String team){
        if(team.equals("BLUE")){
            telemetry.addData("disTogoalXY", getXYdistToPoint(GOAL_BLUE));
            return getXYdistToPoint(GOAL_BLUE);
        }
        telemetry.addData("disTogoalXY", getXYdistToPoint(GOAL_BLUE));
        return getXYdistToPoint(GOAL_RED);
    }

    public double getDistFromPoint(Pose2D point){
        return Math.sqrt(square(getXYdistToPoint(point).first) + square(getXYdistToPoint(point).second));
    }
    public double getDistFromGoal(String team){
        return Math.sqrt(square(getXYdistToGoal(team).first) + square(getXYdistToGoal(team).second));
    }

    public double getAngleFromPoint(Pose2D point){
        return Math.atan2(getXYdistToPoint(point).second, getXYdistToPoint(point).first);
    }
    public double getAngleFromGoal(String team){
        double deg = Math.atan2(getXYdistToGoal(team).second, getXYdistToGoal(team).first);
        return Math.toDegrees(deg);
    }
}
