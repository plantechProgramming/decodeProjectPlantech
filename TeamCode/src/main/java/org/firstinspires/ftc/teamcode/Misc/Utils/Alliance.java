package org.firstinspires.ftc.teamcode.Misc.Utils;

public enum Alliance {
    UNKNOWN,
    BLUE,
    RED;

    private static Alliance currentAlliance = UNKNOWN;

    public static Alliance get(){
        return currentAlliance;
    }

    public static void set(Alliance alliance){
        currentAlliance = alliance;
    }
}
