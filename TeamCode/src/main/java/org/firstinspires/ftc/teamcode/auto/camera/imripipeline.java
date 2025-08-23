package org.firstinspires.ftc.teamcode.auto.camera;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.opencv.core.*;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.List;
@TeleOp(name = "imripipeline")
public class imripipeline extends LinearOpMode {

    public Scalar lower_yellow = new Scalar(20, 50, 50); // Adjusted lower bound
    public Scalar upper_yellow = new Scalar(40, 255, 255);

    public Scalar lower_gray = new Scalar(0, 0, 50); // Gray floor color lower bound
    public Scalar upper_gray = new Scalar(180, 255, 200); // Gray floor color upper bound

    private Mat hsvMat = new Mat();
    private Mat binaryMat = new Mat();
    private Mat grayMask = new Mat();
    private Mat invertedGrayMask = new Mat();
    private Mat maskedInputMat = new Mat();
    private List<MatOfPoint> contours = new ArrayList<>();
    private Telemetry telemetry;

    public imripipeline(Telemetry telemetry) {
        this.telemetry = telemetry;
    }

    public Mat processFrame(Mat input) {
        contours.clear();

        // Convert to HSV color space
        Imgproc.cvtColor(input, hsvMat, Imgproc.COLOR_RGB2HSV);

        // Create yellow mask
        Core.inRange(hsvMat, lower_yellow, upper_yellow, binaryMat);

        // Create gray mask and invert it
        Core.inRange(hsvMat, lower_gray, upper_gray, grayMask);
        Core.bitwise_not(grayMask, invertedGrayMask);

        // Combine masks to isolate yellow objects on non-gray areas
        Core.bitwise_and(binaryMat, invertedGrayMask, binaryMat);

        // Find contours of yellow objects
        Imgproc.findContours(binaryMat, contours, new Mat(), Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);

        // Variables for finding the optimal block
        Rect bestBlock = null;
        double bestScore = Double.MIN_VALUE; // Start with the minimum possible value
        int imageCenterX = input.width() / 2;

        // Iterate through contours
        for (MatOfPoint contour : contours) {
            Rect rect = Imgproc.boundingRect(contour);

            // Ignore contours that are too small or overlap multiple blocks
            if (rect.area() < 500) {
                continue;
            }

            // Draw a rectangle around each yellow block
            Imgproc.rectangle(input, new Point(rect.x, rect.y), new Point(rect.x + rect.width, rect.y + rect.height), new Scalar(0, 255, 0), 2);

            // Calculate the score for the current block (prioritizing higher y values)
            double score = rect.y; // Higher y-values (closer to bottom) are prioritized

            // Update the best block if this one is better
            if (score > bestScore) {
                bestScore = score;
                bestBlock = rect;
            }
        }

        // Process the best block
        if (bestBlock != null) {
            // Draw a filled white rectangle under the best block
            Imgproc.rectangle(input, new Point(bestBlock.x, bestBlock.y), new Point(bestBlock.x + bestBlock.width, bestBlock.y + bestBlock.height), new Scalar(255, 255, 255), -1);
            // Highlight the best block with an orange rectangle
            Imgproc.rectangle(input, new Point(bestBlock.x, bestBlock.y), new Point(bestBlock.x + bestBlock.width, bestBlock.y + bestBlock.height), new Scalar(255, 165, 0), 5);

            // Calculate offsets and angle
            int xOffset = bestBlock.x + bestBlock.width / 2 - imageCenterX;
            int yOffset = input.height() - bestBlock.y;
            double angle = Math.toDegrees(Math.atan2(yOffset, xOffset));

            // Telemetry output
            telemetry.addData("Best Block X Offset", xOffset);
            telemetry.addData("Best Block Y Offset", yOffset);
            telemetry.addData("Best Block Angle", angle);
        }

        telemetry.update();
        return input;
    }

    @Override
    public void runOpMode() throws InterruptedException {

    }
}