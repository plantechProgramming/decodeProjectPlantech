package org.firstinspires.ftc.teamcode.auto.camera;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.TeamOpMode;
import org.firstinspires.ftc.teamcode.teleOp.Utils;
import org.firstinspires.ftc.teamcode.teleOp.actions.Shooter;
import org.firstinspires.ftc.vision.VisionPortal;

/**
 * FILE IS NOT USED. here for "what if we need it for some reason in comp"
 * reasons.
 */
@Config
@TeleOp(name = "aprilTags", group = "camera")
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
        Shooter shooter = new Shooter(shootMotor,dashboardTelemetry,shootMotorOp, odometry);
        Utils utils = new Utils();
        int count = 0;
        int errorCounter = 0;
        double curr = -144;
        double raw;
        double sum = 0;
        while (opModeIsActive()) {
            tagLocalization.detectTags();
            shooter.noPhysShootHomeostasis(0.4);
            if(tagLocalization.goalTag != null){
//                count++;
//                tagLocalization.getCurrDeg(tagLocalization.goalTag);
//                if(count >= loopsPerUpdate){
//                    count = 0;
//                    curr = tagLocalization.getCurrDeg(tagLocalization.goalTag);
//                    tagLocalization.filteredYawPrev = -144.5;
//                    utils.resetSum();
//               }
                telemetry.addData("yaw", tagLocalization.goalTag.ftcPose.yaw);
                telemetry.addData("x", tagLocalization.goalTag.ftcPose.x);
                telemetry.addData("y", tagLocalization.goalTag.ftcPose.y);
                telemetry.addData("cur deg",tagLocalization.getCurrDeg(tagLocalization.goalTag));
                telemetry.addData("xy reloc",tagLocalization.getRelocXY(tagLocalization.goalTag));
                telemetry.addData("xy rotated",tagLocalization.getXYToTag(tagLocalization.goalTag));
                telemetry.addData("wanted deg", Math.toDegrees(Math.atan2(tagLocalization.getXYToTag(tagLocalization.goalTag).second, tagLocalization.getXYToTag(tagLocalization.goalTag).first)));
                telemetry.update();
                dashboardTelemetry.addData("yaw", tagLocalization.goalTag.ftcPose.yaw);
                dashboardTelemetry.addData("x", tagLocalization.goalTag.ftcPose.x);
                dashboardTelemetry.addData("y", tagLocalization.goalTag.ftcPose.y);
                dashboardTelemetry.addData("cur deg",tagLocalization.getCurrDeg(tagLocalization.goalTag));
                dashboardTelemetry.addData("xy reloc",tagLocalization.getRelocXY(tagLocalization.goalTag));
                dashboardTelemetry.addData("xy rotated",tagLocalization.getXYToTag(tagLocalization.goalTag));
                dashboardTelemetry.addData("wanted deg", Math.toDegrees(Math.atan2(utils.GOAL_RED.getY(DistanceUnit.CM) - tagLocalization.getRelocXY(tagLocalization.goalTag).second, utils.GOAL_RED.getX(DistanceUnit.CM) - tagLocalization.getRelocXY(tagLocalization.goalTag).first)));
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