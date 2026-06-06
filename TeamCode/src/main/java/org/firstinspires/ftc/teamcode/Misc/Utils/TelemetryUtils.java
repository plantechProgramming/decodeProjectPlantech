package org.firstinspires.ftc.teamcode.Misc.Utils;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class TelemetryUtils {
    public static <V> void addVar(Telemetry telemetry, V var){
        telemetry.addData(""+var, var);
    }

    public static void addTitle(Telemetry telemetry, String title){
        String dashes = "------------";
        telemetry.addLine(dashes + title + dashes);
    }
}
