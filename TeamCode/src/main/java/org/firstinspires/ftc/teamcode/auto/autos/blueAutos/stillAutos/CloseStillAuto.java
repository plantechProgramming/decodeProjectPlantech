package org.firstinspires.ftc.teamcode.auto.autos.blueAutos.stillAutos;

import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.auto.autos.ReadWrite;

import dev.nextftc.ftc.NextFTCOpMode;


@Autonomous(name = "stillCloseBlue")
public class CloseStillAuto extends NextFTCOpMode {
    private final Pose startPose = new Pose(20.1, 122.5, Math.toRadians(144)); // Start Pose of our robot.

    @Override
    public void onStop(){
        ReadWrite readWrite = new ReadWrite();
        readWrite.writePose(startPose);
    }
}
