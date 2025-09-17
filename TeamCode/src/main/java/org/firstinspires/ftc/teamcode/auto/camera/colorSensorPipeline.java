package org.firstinspires.ftc.teamcode.auto.camera;

import android.graphics.Canvas;
import android.graphics.RecordingCanvas;

import com.sun.tools.javac.util.List;

import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import org.firstinspires.ftc.vision.VisionProcessor;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;

public class colorSensorPipeline implements VisionProcessor {
    private Scalar avgColor = new Scalar(0, 0, 0);
    private Rect square = new Rect();
    public colorSensorPipeline(Rect square){
        this.square = square;
    }

    @Override
    public void init(int width, int height, CameraCalibration calibration) {
    }

    @Override
    public Object processFrame(Mat frame, long captureTimeNanos) {
        Mat roi = frame.submat(square);

        // Compute average BGR color in the ROI
        avgColor = Core.mean(roi);

        // Draw a rectangle on the preview so you can see the sample area
        Imgproc.rectangle(frame, square, new Scalar(0, 255, 0), 2);

        roi.release();
        return null;
    }

    @Override
    public void onDrawFrame(Canvas canvas, int onscreenWidth, int onscreenHeight, float scaleBmpPxToCanvasPx, float scaleCanvasDensity, Object userContext) {

    }

    public Scalar getAvgColor(){
        return avgColor;
    }

    public boolean isColor(List<String> color) {
//        if( avgColor(1) > 100 && color == "RED"){
//            return True;
//        }
        return false;
    }
}
