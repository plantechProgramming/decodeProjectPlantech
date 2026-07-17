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

    public static Pose2D pedroToFTC(Pose pose){
        Pair<Double, Double> scaleXY = new Pair<>(DistanceUnit.CM.fromInches(pose.getX()),
                DistanceUnit.CM.fromInches(pose.getY()));

        Pair<Double, Double> rotScaledXY = PoseFunctions.rotation2D(scaleXY.first,scaleXY.second,90);
        rotScaledXY = new Pair<>(rotScaledXY.first, -rotScaledXY.second);

        Pair<Double, Double> moveRotatedScaledXY = new Pair<>(rotScaledXY.first + PoseFunctions.LEN_FIELD/2,
                rotScaledXY.second + PoseFunctions.LEN_FIELD/2);

        double newHeading = AngleFunctions.angleModulo(Math.toDegrees(pose.getHeading()) + 90);
        return new Pose2D(DistanceUnit.CM, moveRotatedScaledXY.first, moveRotatedScaledXY.second, AngleUnit.DEGREES, newHeading);
    }
}
