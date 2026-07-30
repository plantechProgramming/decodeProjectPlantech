package org.firstinspires.ftc.teamcode.Misc.Utils;

import static org.firstinspires.ftc.teamcode.Misc.Utils.Extras.ROBOT_SIZE;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.canvas.CanvasOp;
import com.acmerobotics.dashboard.canvas.Scale;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

import java.util.ArrayList;
import java.util.List;

public class DashboardCanvas {
    private TelemetryPacket telemetryPacket;

    public DashboardCanvas() {
        telemetryPacket = new TelemetryPacket();
    }
    public DashboardCanvas addPreciseRobot(Pose2D pose) {
        double x = pose.getX(DistanceUnit.INCH);
        double y = pose.getY(DistanceUnit.INCH);
        double heading = pose.getHeading(AngleUnit.RADIANS);
        telemetryPacket.fieldOverlay()
                .setStroke("#FF0000")
                .setStrokeWidth(1)
                .setTranslation(x, y)
                .setRotation(heading)
                .strokeRect(-ROBOT_SIZE.second/2, -ROBOT_SIZE.first/2,
                        ROBOT_SIZE.second, ROBOT_SIZE.first);
        return this;
    }

//    public static void drawRobotPrecisly(Pose2D pose){ // the robot is a represented by a square
//        double x = pose.getX(DistanceUnit.INCH);
//        double y = pose.getY(DistanceUnit.INCH);
//        double heading = pose.getHeading(AngleUnit.RADIANS);
//
//        // TODO: make it so that the square that represents the robot rotates depending on the heading of the robot
//        TelemetryPacket telemetryPacket = new TelemetryPacket();
//        telemetryPacket.fieldOverlay()
//                .setStroke("#FF0000")
//                .setStrokeWidth(1)
//                .setTranslation(x, y)
//                .setRotation(heading)
//                .strokeRect(-ROBOT_SIZE.second/2, -ROBOT_SIZE.first/2,
//                        ROBOT_SIZE.second, ROBOT_SIZE.first);
//        FtcDashboard.getInstance().sendTelemetryPacket(telemetryPacket);
//    }
    private static final double ROBOT_RADIUS = Math.min(ROBOT_SIZE.first, ROBOT_SIZE.second)/2; // in inches
    // the robot is represented as a circle and the angle is shown by a line from the center of the robot
    public DashboardCanvas addRobotAsCircle(Pose2D pose){
        return addRobotAsCircle(pose, "#FF0000");
    }
    public DashboardCanvas addRobotAsCircle(Pose2D pose, String color){
        double x = pose.getX(DistanceUnit.INCH);
        double y = pose.getY(DistanceUnit.INCH);
        double heading = pose.getHeading(AngleUnit.RADIANS);

        double endX = ROBOT_RADIUS/1.5*Math.cos(heading)+x; // the 1.5 is to make the line not go all the way
        double endY = ROBOT_RADIUS/1.5*Math.sin(heading)+y; // the 1.5 is to make the line not go all the way

        telemetryPacket.fieldOverlay()
                .setStroke(color)
                .strokeCircle(x, y, ROBOT_RADIUS)
                .strokeLine(x, y, endX, endY);
        return this;
    }
    public void draw(){
        FtcDashboard.getInstance().sendTelemetryPacket(telemetryPacket);
    }
}
