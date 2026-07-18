package org.firstinspires.ftc.teamcode.auto.autos.blueAutos.stillAutos;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Misc.DataSaving;
import org.firstinspires.ftc.teamcode.Misc.Utils.Alliance;
import org.firstinspires.ftc.teamcode.TeamOpMode;
import org.firstinspires.ftc.teamcode.auto.autos.paths.Paths;


@Autonomous(name = "still Close Blue", group = "still")
public class CloseStillAuto extends TeamOpMode {

    @Override
    public  void postInit(){
        Alliance.set(Alliance.BLUE);
    }
    @Override
    protected void run() {
    }

    @Override
    protected void end() {
        Paths paths = new Paths();
        DataSaving.setEndPos(paths.points.startPose);
    }
}
