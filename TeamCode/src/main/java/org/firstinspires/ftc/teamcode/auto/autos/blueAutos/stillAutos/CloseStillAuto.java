package org.firstinspires.ftc.teamcode.auto.autos.blueAutos.stillAutos;

import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.auto.PathsBlue;
import org.firstinspires.ftc.teamcode.auto.PathsRed;
import org.firstinspires.ftc.teamcode.auto.autos.ReadWrite;

import dev.nextftc.ftc.NextFTCOpMode;


@Autonomous(name = "still Close Blue", group = "still")
public class CloseStillAuto extends NextFTCOpMode {

    PathsBlue path = new PathsBlue();

    ReadWrite readWrite = new ReadWrite();

    @Override
    public void onUpdate() {
        readWrite.writePose(path.getSPose());
    }
}
