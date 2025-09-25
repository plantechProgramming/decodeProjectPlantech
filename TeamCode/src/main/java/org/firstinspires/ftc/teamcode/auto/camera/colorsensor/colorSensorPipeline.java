package org.firstinspires.ftc.teamcode.auto.camera.colorsensor;

import android.graphics.Canvas;

import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import org.firstinspires.ftc.teamcode.auto.camera.Color;
import org.firstinspires.ftc.vision.VisionProcessor;
import org.opencv.core.Core;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;

public class colorSensorPipeline implements VisionProcessor {
    private Scalar avgColor = new Scalar(0, 0, 0);
    private final Rect square;
    public colorSensorPipeline(Rect square){
        this.square = square;
    }

    @Override
    public void init(int width, int height, CameraCalibration calibration) {
    }

    @Override
    public Object processFrame(Mat frame, long captureTimeNanos) {
        Imgproc.cvtColor(frame,frame,Imgproc.COLOR_RGB2HSV);
        Mat roi = frame.submat(square);

        // Compute average BGR color in the ROI
        avgColor = Core.mean(roi);

        // Draw a rectangle on the preview so you can see the sample area
        Imgproc.rectangle(frame, square, new Scalar(0, 255, 0), 2);
        Imgproc.cvtColor(frame,frame,Imgproc.COLOR_HSV2RGB);
        roi.release();
        return null;
    }

    @Override
    public void onDrawFrame(Canvas canvas, int onscreenWidth, int onscreenHeight, float scaleBmpPxToCanvasPx, float scaleCanvasDensity, Object userContext) {

    }

    public Scalar getAvgColor(){
        return avgColor;
    }

    public boolean isColor(Color color) {
        Mat pixelMat = new Mat(1, 1, CvType.CV_8UC3);
        pixelMat.setTo(avgColor);
        Mat mask = new Mat();
        Core.inRange(pixelMat, color.getMin(), color.getMax(), mask);
        return mask.get(0, 0)[0] != 0;
    }

}
