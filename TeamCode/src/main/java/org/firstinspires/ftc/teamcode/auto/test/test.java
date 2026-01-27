package org.firstinspires.ftc.teamcode.auto.test;

import static org.firstinspires.ftc.teamcode.auto.pedro.Tuning.follower;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.auto.PathsBlue;
import org.firstinspires.ftc.teamcode.auto.PathsRed;

import dev.nextftc.extensions.pedro.PedroComponent;

@Autonomous(name="test")
public class test extends LinearOpMode {


    @Override
    public void runOpMode() throws InterruptedException {
        PathsRed pathsRed = new PathsRed();
        telemetry.addData("SPR: ", pathsRed.getSPose());
        telemetry.update();

        sleep(1000000);
    }
    @Override
    public void waitForStart(){

    }

}
