package org.firstinspires.ftc.teamcode.Misc.Utils;

import com.pedropathing.Drivetrain;
import com.pedropathing.follower.Follower;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.subsystems.DriveTrain;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;

import java.util.ArrayList;
import java.util.List;

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

    public static void updatePedroTelemetry(Telemetry telemetry, Follower follower){
        TelemetryUtils.addTitle(telemetry, "starting pedro telemetry");
        telemetry.addData("robot x", follower.getPose().getX()); // in inches
        telemetry.addData("robot y", follower.getPose().getY()); // in inches
        telemetry.addData("robot heading(degrees)", follower.getPose().getY()); // in deg
        TelemetryUtils.addTitle(telemetry, "ending pedro telemetry");
    }
}
