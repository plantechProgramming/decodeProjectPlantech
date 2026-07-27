package org.firstinspires.ftc.teamcode.Misc.Utils;

import android.util.Pair;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.RobotStatus;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.pedropathing.Drivetrain;
import com.pedropathing.follower.Follower;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.subsystems.DriveTrain;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;

import java.util.ArrayList;
import java.util.List;

public class TelemetryUtils {

    public static void addTitle(Telemetry telemetry, String title){
        String dashes = "------------";
        telemetry.addLine(dashes + title + dashes);
    }
    private static final Pair<Double, Double> ROBOT_SIZE = new Pair<>(15.43, 17.32); // width, height in that order, in inches
    public static void setRobotPosToDraw(DistanceUnit distanceUnit, double x, double y, AngleUnit angleUnit, double heading){
        double xInInches = distanceUnit.toInches(x);
        double yInInches = distanceUnit.toInches(y);
        double headingInRedians = angleUnit.toRadians(heading);
        TelemetryPacket telemetryPacket = new TelemetryPacket();
        telemetryPacket.fieldOverlay()
                .setStroke("#FF0000")
                .setStrokeWidth(1)
                .strokeRect(xInInches-ROBOT_SIZE.first/2, yInInches-ROBOT_SIZE.second/2, ROBOT_SIZE.first, ROBOT_SIZE.second);
        FtcDashboard.getInstance().sendTelemetryPacket(telemetryPacket);
    }

    public static void updateCertainTelemtries(Telemetry telemetry, Follower follower, Shooter shooter){
        updatePedroTelemetry(telemetry, follower);
        shooter.updateTelemetry(telemetry);
    }
    public static void updateCertainTelemtries(Telemetry telemetry, DriveTrain drivetrain, Shooter shooter, PoseFunctions poseFunctions){
        drivetrain.updateTelemetry(telemetry);
        shooter.updateTelemetry(telemetry);
        poseFunctions.updateTelemetry(telemetry);
    }

    public static void updateCertainTelemtries(Telemetry telemetry, DriveTrain drivetrain, Shooter shooter, PoseFunctions poseFunctions, Follower follower){
        drivetrain.updateTelemetry(telemetry);
        shooter.updateTelemetry(telemetry);
        poseFunctions.updateTelemetry(telemetry);
        updatePedroTelemetry(telemetry, follower);
    }

    public static void updatePedroTelemetry(Telemetry telemetry, Follower follower){
        TelemetryUtils.addTitle(telemetry, "starting pedro telemetry");
        telemetry.addData("robot x", follower.getPose().getX()); // in inches
        telemetry.addData("robot y", follower.getPose().getY()); // in inches
        telemetry.addData("robot heading(degrees)", follower.getPose().getY()); // in deg
        TelemetryUtils.addTitle(telemetry, "ending pedro telemetry");
    }
}
