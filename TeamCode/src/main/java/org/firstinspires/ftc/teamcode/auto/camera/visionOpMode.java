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
import org.opencv.core.Rect;
import org.opencv.core.Scalar;

@Autonomous(name = "vision")
public class visionOpMode extends LinearOpMode {

    private visionPipeline pipeLine;
    private colorSensorPipeline colorSensor;
    @Override
    public void runOpMode() throws InterruptedException {
        Rect rect = new Rect(150, 100, 50, 50);

        pipeLine = new visionPipeline(Color.YELLOW);
        colorSensor = new colorSensorPipeline(rect);

        waitForStart();

//        visionPortal = VisionPortal.easyCreateWithDefaults(hardwareMap.get(WebcamName.class, "Webcam"),
//        pipeLine);


        VisionPortal portal = new VisionPortal.Builder()
//                .addProcessor(pipeLine)
                .addProcessor(colorSensor)
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
