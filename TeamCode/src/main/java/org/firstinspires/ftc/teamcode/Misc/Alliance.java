package org.firstinspires.ftc.teamcode.Misc;

public enum Alliance {
    UNKNOWN("NONE"),
    BLUE("BLUE"),
    RED("RED");
    private final String team;
    Alliance(String desc){
        this.team=desc;
    }

    public String getTeam() {
        return this.team;
    }
}
