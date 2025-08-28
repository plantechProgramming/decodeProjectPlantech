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
        Mat mask = new Mat();
        Mat maskApplied = new Mat();
        Mat onlyEdges = new Mat();
        state = 0;

        this.makeMask(frame,mask);
        this.applyMask(frame,mask,maskApplied);
        this.detectEdge(maskApplied,onlyEdges);
        Imgproc.cvtColor(onlyEdges,frame, Imgproc.COLOR_GRAY2RGB);
        return null;
    }

    @Override
    public void onDrawFrame(Canvas canvas, int onscreenWidth, int onscreenHeight, float scaleBmpPxToCanvasPx, float scaleCanvasDensity, Object userContext) {

    }

    public void makeMask(Mat input, Mat output){
        Mat frameHSV = new Mat();

        Imgproc.cvtColor(input,frameHSV,Imgproc.COLOR_RGB2HSV);
        Core.inRange(frameHSV,min,max,output);
    }

    public void applyMask(Mat frame, Mat mask, Mat output){
        Imgproc.cvtColor(frame, frame, Imgproc.COLOR_RGB2HSV);
        Core.bitwise_and(frame, frame, output, mask);
    }
    public void detectEdge(Mat input, Mat output){
        Imgproc.cvtColor(input,input,Imgproc.COLOR_HSV2RGB);
        Imgproc.cvtColor(input,input,Imgproc.COLOR_RGB2GRAY);
        Imgproc.Canny(input,output,100,200);
    }

}
