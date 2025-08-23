package org.firstinspires.ftc.teamcode.auto.camera;

import android.graphics.Canvas;

import com.qualcomm.robotcore.hardware.ColorRangeSensor;

import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import org.firstinspires.ftc.vision.VisionProcessor;
import org.firstinspires.ftc.vision.opencv.ColorRange;
import org.firstinspires.ftc.vision.opencv.ColorSpace;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfInt;
import org.opencv.core.MatOfPoint;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.RotatedRect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;

import java.util.ArrayList;
import java.util.List;


public class visionPipeline implements VisionProcessor {
    public ColorRange colorRange;
    int x = 75;
    Scalar max;
    Scalar min;
    int state = 0;

    public visionPipeline(Scalar min, Scalar max){
        this.colorRange = new ColorRange(ColorSpace.HSV,min,max);
        this.max = max;
        this.min = min;

    }

    @Override
    public void init(int width, int height, CameraCalibration calibration) {

    }

    @Override
    public Object processFrame(Mat frame, long captureTimeNanos) {

        // TODO: init mats better
        Mat mat0 = new Mat();
        Mat mat1 = new Mat();
        Mat mat2 = new Mat();
        state = 0;

        // mat_i is the input for case i, mat i+1 is output
        switch (state){
            case 0:
                // make black if not in range, white otherwise
                Imgproc.cvtColor(frame,mat0,Imgproc.COLOR_RGB2HSV);
                Core.inRange(mat0,min,max,mat1);
                state = 1;
            case 1:
                // make previous whites the initial color
                Imgproc.cvtColor(frame, frame,Imgproc.COLOR_RGB2HSV);
                Core.bitwise_and(frame, frame,mat2,mat1); // frame && frame == frame, but on it you use the mask
                state = 2;
            default:
                Imgproc.cvtColor(mat2,frame,Imgproc.COLOR_HSV2RGB);
                break;
        }

        return null;
    }

    @Override
    public void onDrawFrame(Canvas canvas, int onscreenWidth, int onscreenHeight, float scaleBmpPxToCanvasPx, float scaleCanvasDensity, Object userContext) {

    }

    public int getState(){
        return state;
    }
}
