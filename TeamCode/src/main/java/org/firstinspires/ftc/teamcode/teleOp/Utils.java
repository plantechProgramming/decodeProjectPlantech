package org.firstinspires.ftc.teamcode.teleOp;

import android.util.Pair;

import com.pedropathing.geometry.Pose;
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
    public Utils(){}
    public final double LEN_FIELD = 360.172; // in cm
    double yOffset = 20;//prev = 18
    double xOffset = 20; // prev = 16
    public final Pose2D GOAL_RED = new Pose2D(DistanceUnit.CM,-LEN_FIELD/2-xOffset, -LEN_FIELD/2 - yOffset,AngleUnit.DEGREES,0);
    public final Pose2D GOAL_BLUE = new Pose2D(DistanceUnit.CM,LEN_FIELD/2-xOffset, -LEN_FIELD/2 + yOffset,AngleUnit.DEGREES,0);

    public Pair<Double, Double> getXYdistToPoint(Pose2D point){
        double robotX = odometry.getPosX(DistanceUnit.CM);
        double robotY = odometry.getPosY(DistanceUnit.CM);
        double pointX = point.getX(DistanceUnit.CM);
        double pointY = point.getY(DistanceUnit.CM);
        return new Pair<>(pointX-robotX, pointY-robotY);
    }
    public double getPointToGoalAngle(Pose2D point1, String team){
        double x1 = point1.getX(DistanceUnit.CM);
        double y1 = point1.getY(DistanceUnit.CM);
        double goalX = getTeamGoal(team).getX(DistanceUnit.CM);
        double goalY = getTeamGoal(team).getY(DistanceUnit.CM);
        return Math.toDegrees(Math.atan2(goalY-y1,goalX-x1));
    }
    public Pose2D getTeamGoal(String team){
        if(team.equals("BLUE")) return GOAL_BLUE;
        if(team.equals("RED")) return GOAL_RED;
        return null;
    }
    public Pair<Double,Double> getXYdistToGoal(String team){
        if(team.equals("BLUE")){
            return getXYdistToPoint(GOAL_BLUE);
        }
        return getXYdistToPoint(GOAL_RED);
    }
    public double getDistFromPoint(Pose2D point){
        return Math.hypot(getXYdistToPoint(point).first, getXYdistToPoint(point).second);
    }
    public double getDistFromGoal(String team){
        return Math.hypot(getXYdistToGoal(team).first,getXYdistToGoal(team).second);
    }

    public double getAngleFromPoint(Pose2D point){
        return Math.toDegrees(Math.atan2(getXYdistToPoint(point).second, getXYdistToPoint(point).first));
    }
    public double getAngleFromGoal(String team){
        double deg = Math.atan2(getXYdistToGoal(team).second, getXYdistToGoal(team).first);
        return Math.toDegrees(deg);
    }
    public Pose2D PedroPoseConverter(Pose pose){
        double x = pose.getX();
        double y = pose.getY();
        double hed = Math.toDegrees(pose.getHeading());
        double lenField = 365.76; // 144 inch to cm
        double newx = ((-lenField/144)*x)+lenField/2;
        double newy = ((-lenField/144)*y)+lenField/2;
        hed = hed - 180;
        if(hed <= 180){
            hed += 360;
        }
        return new Pose2D(DistanceUnit.CM, newx, newy, AngleUnit.DEGREES, hed);
    }

    public double angleModulo(double num){
        // fuck java * TREE(3)
        // java doesnt do mod well for negatives and doubles, so:
        // add 360 to normalize to [0,360)
        // multiply by 100 to pretend its an int, then divide to get the right val
        return ((num +360)*100%(360*100))/100;
    }

    public double getDistBetweenAngles(double alpha, double beta){
        double currentError = alpha - beta;
        if (currentError < -180){
            currentError += 360;
        }
        if (currentError > 180){
            currentError -= 360;
        }
        return currentError;
    }
}
