package org.firstinspires.ftc.teamcode.Tests.Auto; // make sure this aligns with class location
import com.pedropathing.follower.Follower;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Misc.Utils.Alliance;
import org.firstinspires.ftc.teamcode.Misc.Utils.Extras;
import org.firstinspires.ftc.teamcode.auto.autos.paths.Paths;
import org.firstinspires.ftc.teamcode.auto.autos.paths.Points;
import org.firstinspires.ftc.teamcode.auto.pedro.Constants;

import java.util.Arrays;

@Autonomous(group = "autonomous tests")
public class testPedro extends LinearOpMode {
    private Follower follower;
    Paths path;
    Extras extras;
    ElapsedTime elapsedTime;
    int counter = 0;
    public void autonomousPathUpdate() {
        if(counter == 0){
            follower.followPath(path.scorePreloadFar);
            counter++;
        } else if (counter == 1) {
            follower.followPath(path.grabGPPFar);
            counter++;
        }
    }

    @Override
    public void runOpMode() throws InterruptedException {
        extras = new Extras();
        elapsedTime = new ElapsedTime();
        extras.startHistogram(1000, 0.1);
        Alliance.set(Alliance.BLUE);
        path = new Paths();
        follower = Constants.createFollower(hardwareMap);

        follower.setStartingPose(path.points.startPoseFar);
        path.buildPaths(follower);
        waitForStart();
        while (opModeIsActive()){
            elapsedTime.reset();
            // These loop the movements of the robot, these must be called continuously in order to work
            follower.update();
            if(!follower.isBusy())
                autonomousPathUpdate();

            // Feedback to Driver Hub for debugging
            telemetry.addData("x", follower.getPose().getX());
            telemetry.addData("y", follower.getPose().getY());
            telemetry.addData("heading", follower.getPose().getHeading());
            telemetry.update();
            extras.updateHistogram(elapsedTime.milliseconds());
        }
        System.out.println("auto loop time: " + Arrays.toString(extras.getHistogram()));
    }

}