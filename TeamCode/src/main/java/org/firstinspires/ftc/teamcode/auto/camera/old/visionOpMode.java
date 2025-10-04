package org.firstinspires.ftc.teamcode.auto.camera.old;


import android.util.Size;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.auto.camera.Color;
import org.firstinspires.ftc.teamcode.auto.camera.colorsensor.colorSensorPipeline;
import org.firstinspires.ftc.vision.VisionPortal;
import org.opencv.core.Rect;

import java.util.ArrayList;
import java.util.List;

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

     //   visionPortal = VisionPortal.easyCreateWithDefaults(hardwareMap.get(WebcamName.class, "Webcam"),
//        pipeLine);


        VisionPortal portal = new VisionPortal.Builder()
//                .addProcessor(pipeLine)
                .addProcessor(colorSensor)
                .setCameraResolution(new Size(320, 240))
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam"))
                .build();

        List<Color> colors = new ArrayList<Color>();
        colors.add(Color.YELLOW);
        // needed to run continuously
        while (opModeIsActive() || opModeInInit()) {
            telemetry.addData("color",colorSensor.isColor(Color.YELLOW));
            telemetry.addLine("hello world");
            telemetry.update();
            sleep(20);
        }
//        visionPortal.close();
    }
}
