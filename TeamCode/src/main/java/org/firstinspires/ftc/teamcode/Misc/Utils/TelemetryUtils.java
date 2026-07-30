package org.firstinspires.ftc.teamcode.Misc.Utils;

import static org.firstinspires.ftc.teamcode.Misc.Utils.Extras.ROBOT_SIZE;

import android.util.Pair;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.pedropathing.follower.Follower;
import com.qualcomm.robotcore.hardware.I2cDeviceSynchReadHistoryImpl;

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
