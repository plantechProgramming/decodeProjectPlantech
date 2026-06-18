package org.firstinspires.ftc.teamcode.auto.autos.redAutos.stillAutos;

import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.auto.PathsRed;
import org.firstinspires.ftc.teamcode.auto.autos.ReadWrite;

import dev.nextftc.ftc.NextFTCOpMode;

@Autonomous(name = "still Far Red", group = "still")
public class FarStillAuto extends NextFTCOpMode {
    private final Pose startPose = new Pose(56.15, 8.4, Math.toRadians(90)).mirror(); // Start Pose of our robot.

    PathsRed path = new PathsRed();

    ReadWrite readWrite = new ReadWrite();

    @Override
    public void onUpdate() {
        readWrite.writePose(path.getSPoseFar());
    }
}
