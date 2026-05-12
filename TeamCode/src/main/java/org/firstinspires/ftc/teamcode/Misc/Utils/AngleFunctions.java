package org.firstinspires.ftc.teamcode.Misc.Utils;

import android.util.Pair;

public class AngleFunctions {

    public double convertToWrapAroundAngle(double angle){
        if (angle < -180){
            angle += 360;
        }
        if (angle > 180){
            angle -= 360;
        }
        return angle;
    }

    public double getDiffBetweenAngles(double wanted, double current){// positive = left, negative = right// gyro coordinate system
        double currentError = wanted - current;
        return convertToWrapAroundAngle(currentError);
    }

    public double getLongestDiffBetweenAngles(double wanted, double current){// positive = left, negative = right // gyro coordinate system
        double shortestPath = getDiffBetweenAngles(wanted, current);
        if(shortestPath < 0){
            return 360 + shortestPath;
        }
        else{
            return shortestPath - 360;
        }
    }

    public double convertGyroAngleTo360(double deg){// gets bigger to the left
        double newDeg = deg;
        if(deg<0){
            newDeg = 360 + deg;
        }
        return newDeg;
    }

    public double angleModulo(double num){
        // fuck java * TREE(3)
        // java doesnt do mod well for negatives and doubles, so:
        // add 360 to normalize to [0,360)
        // multiply by 100 to pretend its an int, then divide to get the right val
        return ((num +360)*100%(360*100))/100;
    }

    public Pair<Double, Double> rotation2D(double x, double y, double deg){
        double rad = Math.toRadians(deg);
        return new Pair<>(x*Math.cos(rad)-y*Math.sin(rad), x*Math.sin(rad)+y*Math.cos(rad));
    }
}
