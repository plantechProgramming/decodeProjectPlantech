package org.firstinspires.ftc.teamcode.Misc.Utils;

import android.util.Pair;

import com.pedropathing.geometry.Pose;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.R;

public class Converters {
    public static Pose2D PedroPoseConverter(Pose pose){ // from pedro cords to the old bad Pinpoint cords
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

    // the following function doesn't work if you want to do the conversion use pedro's built in coordinate system convertors
//    public static Pose2D pedroToFTC(Pose pose){
//        double scaledX = DistanceUnit.CM.fromInches(pose.getX());
//        double scaledY = DistanceUnit.CM.fromInches(pose.getY());
//
//        Pair<Double, Double> rotatedXY = PoseFunctions.rotation2D(scaledX,scaledY,90);
//        double newX = rotatedXY.first + PoseFunctions.LEN_FIELD/2;
//        double newY = rotatedXY.second + PoseFunctions.LEN_FIELD/2;
//
//        double newHeading = AngleFunctions.angleModulo(Math.toDegrees(pose.getHeading()) + 90);
//        return new Pose2D(DistanceUnit.CM, newX, newY, AngleUnit.DEGREES, newHeading);
//    }
}
