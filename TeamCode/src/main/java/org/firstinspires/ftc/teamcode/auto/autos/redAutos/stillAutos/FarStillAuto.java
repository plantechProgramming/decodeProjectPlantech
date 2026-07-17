package org.firstinspires.ftc.teamcode.auto.autos.redAutos.stillAutos;

import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Misc.DataSaving;
import org.firstinspires.ftc.teamcode.Misc.Utils.Alliance;
import org.firstinspires.ftc.teamcode.TeamOpMode;
import org.firstinspires.ftc.teamcode.auto.TeamAuto;
import org.firstinspires.ftc.teamcode.auto.autos.paths.Paths;


@Autonomous(name = "still Far Red", group = "still")
public class FarStillAuto extends TeamOpMode {
    @Override
    public void postInit(){
        Alliance.set(Alliance.RED);
    }

    @Override
    protected void run() {

    }

    @Override
    protected void end() {
        Paths paths = new Paths();
        DataSaving.setEndPos(paths.points.startPoseFar);
    }
}
