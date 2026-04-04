package org.firstinspires.ftc.teamcode.auto.camera;

import android.widget.Toast;

import androidx.annotation.NonNull;
import androidx.annotation.Nullable;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.CameraName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.OpMode;
import org.firstinspires.ftc.teamcode.teleOp.Utils;
import org.firstinspires.ftc.teamcode.teleOp.actions.DriveTrain;
import org.firstinspires.ftc.teamcode.teleOp.actions.Shooter;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagGameDatabase;
import org.firstinspires.ftc.vision.apriltag.AprilTagMetadata;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.openftc.easyopencv.Util;

import java.lang.reflect.Array;
import java.util.ArrayList;
import java.util.Collection;
import java.util.Collections;
import java.util.Iterator;
import java.util.List;
import java.util.ListIterator;

/**
 * FILE IS NOT USED. here for "what if we need it for some reason in comp"
 * reasons.
 */
@Config
@TeleOp(name = "aprilTags", group = "camera")
public class aprilTagsTest  extends OpMode {
    AprilTagLocalization tagLocalization = new AprilTagLocalization("RED", telemetry); //TODO: change here for red

    public static int loopsPerUpdate = 50;

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
            shooter.noPhysShootHomeostasis(0.5);
            if(tagLocalization.goalTag != null){
                count++;
                tagLocalization.getCurrDeg(tagLocalization.goalTag);
                if(count >= loopsPerUpdate){
                    count = 0;
                    curr = tagLocalization.getCurrDeg(tagLocalization.goalTag);
                    tagLocalization.filteredYawPrev = -144.5;
                    utils.resetSum();
                }
//                telemetry.addData("yaw", tagLocalization.goalTag.ftcPose.yaw);
//                telemetry.addData("x", tagLocalization.goalTag.ftcPose.x);
//                telemetry.addData("y", tagLocalization.goalTag.ftcPose.y);
//                telemetry.addData("cur deg",tagLocalization.getCurrDeg(tagLocalization.goalTag));
//                telemetry.addData("xy reloc",tagLocalization.getRelocXY(tagLocalization.goalTag));
//                telemetry.addData("xy rotated",tagLocalization.getXYToTag(tagLocalization.goalTag));
//                telemetry.update();
            }
            else{
                errorCounter++;
            }
            dashboardTelemetry.addData("error Counter", errorCounter);
            dashboardTelemetry.addData("wanted", -144.5);
            dashboardTelemetry.addData("current", curr);
            dashboardTelemetry.addData("count", count);
            dashboardTelemetry.update();

            sleep(10);
        }
    }


    @Override
    protected void end() {

    }
}