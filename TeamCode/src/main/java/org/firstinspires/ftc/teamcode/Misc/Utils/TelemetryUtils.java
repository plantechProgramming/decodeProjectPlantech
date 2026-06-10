package org.firstinspires.ftc.teamcode.Misc.Utils;

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
    public static <V> void addVar(Telemetry telemetry, V var){
        telemetry.addData(""+var, var);
    }

    public static void addTitle(Telemetry telemetry, String title){
        String dashes = "------------";
        telemetry.addLine(dashes + title + dashes);
    }

    public static <V> void addVarToAllTelemetries(V var){
        for(Telemetry telemetry : telemetries){
            addVar(telemetry, var);
        }
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
}
