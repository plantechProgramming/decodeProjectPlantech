package org.firstinspires.ftc.teamcode.auto.autos.blueAutos.stillAutos;

import static dev.nextftc.extensions.pedro.PedroComponent.follower;

import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.auto.autos.ReadWrite;

import dev.nextftc.ftc.NextFTCOpMode;

@Autonomous(name = "stillFarBlue")
public class FarStillAuto extends NextFTCOpMode {
    final Pose startPose = new Pose(62, 8, Math.toRadians(90)); // Start Pose of our robot.

    @Override
    public void onStop(){
        ReadWrite readWrite = new ReadWrite();
        readWrite.writePose(startPose);
    }
}
