package org.firstinspires.ftc.teamcode.teleOp;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.auto.camera.ColorSensorTest;
import org.firstinspires.ftc.teamcode.teleOp.actions.DriveTrain;
import org.firstinspires.ftc.teamcode.teleOp.actions.Elevator;
import org.firstinspires.ftc.teamcode.OpMode;
import org.firstinspires.ftc.teamcode.auto.camera.aprilTagsTest;
import android.util.Size;

import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagMetadata;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.opencv.ColorBlobLocatorProcessor;
import org.firstinspires.ftc.vision.opencv.ColorRange;
import org.firstinspires.ftc.vision.opencv.ImageRegion;
import org.firstinspires.ftc.vision.opencv.PredominantColorProcessor;
import org.opencv.core.RotatedRect;

import java.util.List;


@Config
@com.qualcomm.robotcore.eventloop.opmode.TeleOp
public class TeleOp extends OpMode {
    @Override
    protected void postInit() {
        Imu.resetYaw();
    }

    @Override
    public void run(){
        DriveTrain driveTrain = new DriveTrain(DriveBackRight, DriveBackLeft, DriveFrontRight, DriveFrontLeft, telemetry, Imu);
        Elevator lift = new Elevator(EA, EH, intake_center_angle,IntakeL,IntakeR, telemetry);
        ColorSensorTest cSencor = new ColorSensorTest();
        cSencor.init(hardwareMap);
        boolean is_up = false;
        aprilTagsTest tags = new aprilTagsTest();
        VisionPortal visionPortal;

        double forward; //-1 to 1
        double turn;
        double drift;
        double botHeading;
        boolean slow = false;
        EH.setDirection(DcMotorSimple.Direction.REVERSE);
        EH.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        EA.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


        while (opModeIsActive() ) {
            forward = -gamepad1.left_stick_y;
            turn = gamepad1.right_stick_x;
            drift = gamepad1.left_stick_x;
            botHeading = Imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

            ElapsedTime elapsedTime = new ElapsedTime();
            driveTrain.drive(forward, drift, turn, botHeading, 1);
            telemetry.addData("x", DriveFrontRight.getCurrentPosition());
            telemetry.addData("y",DriveBackLeft.getCurrentPosition());
            tags.initAprilTag();



            if(gamepad1.x && !slow){
                driveTrain.drive(forward, drift, turn, botHeading, 0.5);
                slow = true;
            }else if(gamepad1.x && slow) {
                driveTrain.drive(forward, drift, turn, botHeading, 1);
                slow = false;

            } telemetry.addData("y: ", DriveBackLeft.getCurrentPosition());
            telemetry.addData("x:", DriveFrontRight.getCurrentPosition());
            if (cSencor.getDetectedColor(telemetry) == ColorSensorTest.DetectedColor.RED){lift.intakefunc(true);}
            else{lift.intakefunc(false);}

            if (tags.Order == "GPP"){
                lift.intakefunc(true);
            }


            if(gamepad1.back){Imu.resetYaw();}

            telemetry.addData("ea",EA.getCurrentPosition());
            telemetry.addData("recognized color: ", cSencor.getDetectedColor(telemetry));
            telemetry.update();
        }
        EH.setPower(0);


    }

    @Override
    protected void end() {

    }

}
