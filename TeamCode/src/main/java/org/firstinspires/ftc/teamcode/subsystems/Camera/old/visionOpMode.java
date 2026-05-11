package org.firstinspires.ftc.teamcode.subsystems.Camera.old;


import android.util.Size;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.Misc.Colors;
import org.firstinspires.ftc.teamcode.subsystems.Camera.colorSensorPipeline;
import org.firstinspires.ftc.vision.VisionPortal;
import org.opencv.core.Rect;

import java.util.ArrayList;
import java.util.List;
@Disabled
@Autonomous(name = "vision",group = "camera")
public class visionOpMode extends LinearOpMode {

    private visionPipeline pipeLine;
    private colorSensorPipeline colorSensor;
    @Override
    public void runOpMode() throws InterruptedException {
        Rect rect = new Rect(150, 100, 50, 50);

        pipeLine = new visionPipeline(Colors.YELLOW);
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

        List<Colors> colors = new ArrayList<Colors>();
        colors.add(Colors.YELLOW);
        // needed to run continuously
        while (opModeIsActive() || opModeInInit()) {
            telemetry.addData("color",colorSensor.isColor(Colors.YELLOW));
            telemetry.addLine("hello world");
            telemetry.update();
            sleep(20);
        }
//        visionPortal.close();
    }
}
