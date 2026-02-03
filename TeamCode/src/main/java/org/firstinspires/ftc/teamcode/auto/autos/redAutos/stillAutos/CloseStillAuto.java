package org.firstinspires.ftc.teamcode.auto.autos.redAutos.stillAutos;

import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.auto.PathsRed;
import org.firstinspires.ftc.teamcode.auto.autos.ReadWrite;

import dev.nextftc.ftc.NextFTCOpMode;


@Autonomous(name = "still Close Red", group = "still")
public class CloseStillAuto extends NextFTCOpMode {
    private final Pose startPose = new Pose(20.1, 122.5, Math.toRadians(144)).mirror(); // Start Pose of our robot.

    @Override
    public void onStop(){
        PathsRed path = new PathsRed();
        ReadWrite readWrite = new ReadWrite();
        readWrite.writePose(path.getSPose());
    }
}
