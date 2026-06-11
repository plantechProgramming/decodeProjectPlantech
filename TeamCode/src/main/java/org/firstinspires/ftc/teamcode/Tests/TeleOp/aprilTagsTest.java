package org.firstinspires.ftc.teamcode.Tests.TeleOp;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.TeamOpMode;
import org.firstinspires.ftc.teamcode.subsystems.Camera.AprilTagLocalization;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;
import org.firstinspires.ftc.vision.VisionPortal;

/**
 * FILE IS NOT USED. here for "what if we need it for some reason in comp"
 * reasons.
 */
@Config
@Disabled
@TeleOp(group = "tests")
public class aprilTagsTest  extends TeamOpMode {
    AprilTagLocalization tagLocalization = new AprilTagLocalization("RED", telemetry); //TODO: change here for red

    public static int loopsPerUpdate = 150;

    @Override
    public void run() {
        tagLocalization.initProcessor(hardwareMap);
        while (tagLocalization.visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING){
            sleep(20);
        }
        tagLocalization.applySettings();
        Shooter shooter = new Shooter();
        int count = 0;
        int errorCounter = 0;
        double curr = -144;
        double raw;
        double sum = 0;
        while (opModeIsActive()) {
            tagLocalization.detectTags();
            shooter.setShooterPowerAsCommand(0.4);
            if(tagLocalization.goalTag != null){
//                count++;
//                tagLocalization.getCurrDeg(tagLocalization.goalTag);
//                if(count >= loopsPerUpdate){
//                    count = 0;
//                    curr = tagLocalization.getCurrDeg(tagLocalization.goalTag);
//                    tagLocalization.filteredYawPrev = -144.5;
//                    utils.resetSum();
//               }
                tagLocalization.setCameraTelemetry(telemetry);
                tagLocalization.setCameraTelemetry(dashboardTelemetry);

                telemetry.update();
                dashboardTelemetry.update();
            }
            else{
                errorCounter++;
            }

        }
    }


    @Override
    protected void end() {

    }
}