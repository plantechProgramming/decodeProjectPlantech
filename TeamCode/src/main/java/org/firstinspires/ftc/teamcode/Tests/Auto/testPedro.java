package org.firstinspires.ftc.teamcode.Tests.Auto; // make sure this aligns with class location
import com.pedropathing.follower.Follower;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Misc.Utils.Alliance;
import org.firstinspires.ftc.teamcode.auto.autos.paths.Paths;
import org.firstinspires.ftc.teamcode.auto.autos.paths.Points;
import org.firstinspires.ftc.teamcode.auto.pedro.Constants;

@Autonomous(group = "autonomous tests")
public class testPedro extends LinearOpMode {
    private Follower follower;
    Paths path;
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
        Alliance.set(Alliance.BLUE);
        path = new Paths();
        follower = Constants.createFollower(hardwareMap);

        follower.setStartingPose(path.points.startPoseFar);
        path.buildPaths(follower);
        waitForStart();
        while (opModeIsActive()){
            // These loop the movements of the robot, these must be called continuously in order to work
            follower.update();
            if(!follower.isBusy())
                autonomousPathUpdate();

            // Feedback to Driver Hub for debugging
            telemetry.addData("x", follower.getPose().getX());
            telemetry.addData("y", follower.getPose().getY());
            telemetry.addData("heading", follower.getPose().getHeading());
            telemetry.update();
        }
    }

}