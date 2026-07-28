package org.firstinspires.ftc.teamcode.Misc.Utils;

import static org.firstinspires.ftc.teamcode.Misc.Utils.Extras.ROBOT_SIZE;

import android.util.Pair;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.pedropathing.follower.Follower;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.Misc.RobotPose;
import org.firstinspires.ftc.teamcode.subsystems.DriveTrain;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;

public class TelemetryUtils {

    public static void addTitle(Telemetry telemetry, String title){
        String dashes = "------------";
        telemetry.addLine(dashes + title + dashes);
    }
    public static void drawRobotPrecisly(Pose2D pose){ // the robot is a represented by a square
        double x = pose.getX(DistanceUnit.INCH);
        double y = pose.getY(DistanceUnit.INCH);
        double heading = pose.getHeading(AngleUnit.RADIANS);
        // TODO: make it so that the square that represents the robot rotates depending on the heading of the robot
        TelemetryPacket telemetryPacket = new TelemetryPacket();
        telemetryPacket.fieldOverlay()
                .setStroke("#FF0000")
                .setStrokeWidth(1)
                .strokeRect(x -ROBOT_SIZE.first/2, y -ROBOT_SIZE.second/2,
                        ROBOT_SIZE.first, ROBOT_SIZE.second);
        FtcDashboard.getInstance().sendTelemetryPacket(telemetryPacket);
    }
    private static final double ROBOT_RADIUS = Math.min(ROBOT_SIZE.first, ROBOT_SIZE.second)/2; // in inches
    // the robot is represented as a circle and the angle is shown by a line from the center of the robot
    public static void drawRobotAsCircle(Pose2D pose){
        double x = pose.getX(DistanceUnit.INCH);
        double y = pose.getY(DistanceUnit.INCH);
        double heading = pose.getHeading(AngleUnit.RADIANS);

        double endX = ROBOT_RADIUS/1.5*Math.cos(heading); // the 1.5 is to make the line not go all the way
        double endY = ROBOT_RADIUS/1.5*Math.sin(heading); // the 1.5 is to make the line not go all the way
        TelemetryPacket telemetryPacket = new TelemetryPacket();
        telemetryPacket.fieldOverlay()
                .setStroke("#FF0000")
                .strokeCircle(x, y, ROBOT_RADIUS)
                .strokeLine(x, y, endX, endY);
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
