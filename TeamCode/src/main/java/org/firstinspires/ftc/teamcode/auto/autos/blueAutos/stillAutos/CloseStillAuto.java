package org.firstinspires.ftc.teamcode.auto.autos.blueAutos.stillAutos;

import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.auto.PathsBlue;
import org.firstinspires.ftc.teamcode.auto.autos.ReadWrite;

import dev.nextftc.ftc.NextFTCOpMode;


@Autonomous(name = "stillCloseBlue")
public class CloseStillAuto extends NextFTCOpMode {

    @Override
    public void onStop(){
        PathsBlue path = new PathsBlue();
        ReadWrite readWrite = new ReadWrite();
        readWrite.writePose(path.getSPose());
    }
}
