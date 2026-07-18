package org.firstinspires.ftc.teamcode.Misc.Utils;

import com.pedropathing.follower.Follower;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.ArrayList;
import java.util.List;

public class TelemetryUtils {

    private static ArrayList<Telemetry> telemetries;

    public static void setTelemetries(ArrayList<Telemetry> telemetries){
        TelemetryUtils.telemetries = new ArrayList<>(telemetries);
    }

    public static void addTelemetry(Telemetry telemetry){
        telemetries.add(telemetry);
    }

    public static void addTitle(Telemetry telemetry, String title){
        String dashes = "------------";
        telemetry.addLine(dashes + title + dashes);
    }

    public static void addTitleToAllTelemetries(String title){
        for(Telemetry telemetry : telemetries){
            addTitle(telemetry, title);
        }
    }

    public static void updateAll(){
        for(Telemetry telemetry : telemetries){
            telemetry.update();
        }
    }

    public static void updatePedroTelemetry(Telemetry telemetry, Follower follower){
        TelemetryUtils.addTitle(telemetry, "starting pedro telemetry");
        telemetry.addData("robot x", follower.getPose().getX()); // in inches
        telemetry.addData("robot y", follower.getPose().getY()); // in inches
        telemetry.addData("robot heading(degrees)", follower.getPose().getY()); // in deg
        TelemetryUtils.addTitle(telemetry, "ending pedro telemetry");
    }
}
