package org.firstinspires.ftc.teamcode.auto.camera;


import android.util.Size;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.opencv.ColorRange;
import org.opencv.core.Scalar;

@TeleOp(name = "grayscale")
public class visionOpMode extends LinearOpMode {
    public int thresh = 100;
    private visionPipeline pipeLine;
    @Override
    public void runOpMode() throws InterruptedException {
        Scalar min = new Scalar( 32, 176,  0);
        Scalar max = new Scalar(255, 255, 132);

        pipeLine = new visionPipeline(min,max);

        waitForStart();

        VisionPortal portal = new VisionPortal.Builder()
                .addProcessor(pipeLine)
                .setCameraResolution(new Size(320, 240))
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam"))
                .build();

        // needed to run continuously
        while (opModeIsActive() || opModeInInit()) {
            telemetry.addLine("Area Density Aspect Arc Circle Center");
            telemetry.update();
            sleep(20);
        }

    }
}
