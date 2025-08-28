package org.firstinspires.ftc.teamcode.auto.camera;


import android.util.Size;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.opencv.ColorRange;
import org.firstinspires.ftc.vision.opencv.ImageRegion;
import org.firstinspires.ftc.vision.opencv.PredominantColorProcessor;
import org.opencv.core.Scalar;

@Autonomous(name = "vision")
public class visionOpMode extends LinearOpMode {
//    public int thresh = 100;
//    private VisionPortal visionPortal;
    private visionPipeline pipeLine;
    @Override
    public void runOpMode() throws InterruptedException {

        // yellow - in opencv hsv
        // opencv hsv != normal hsv, h:0-179, s:0-255,v:0-255
        Scalar minYellow = new Scalar(16, 150, 99);
        Scalar maxYellow = new Scalar(40, 255, 255);

        // red
        Scalar minRed = new Scalar(112, 153, 230);
        Scalar maxRed = new Scalar(190, 230, 255);


        pipeLine = new visionPipeline(minYellow,maxYellow);

        waitForStart();

//        visionPortal = VisionPortal.easyCreateWithDefaults(hardwareMap.get(WebcamName.class, "Webcam"),
//        pipeLine);


        VisionPortal portal = new VisionPortal.Builder()
                .addProcessor(pipeLine)
                .setCameraResolution(new Size(320, 240))
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam"))
                .build();

        // needed to run continuously
        while (opModeIsActive() || opModeInInit()) {
//            telemetry.addLine("Area Density Aspect Arc Circle Center");
            telemetry.update();
            sleep(20);
        }
//        visionPortal.close();
    }
}
