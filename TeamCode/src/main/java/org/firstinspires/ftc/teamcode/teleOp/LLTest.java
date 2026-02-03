package org.firstinspires.ftc.teamcode.teleOp;

import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.OpMode;
import org.firstinspires.ftc.teamcode.teleOp.actions.DriveTrain;
import org.opencv.android.Utils;

@TeleOp
public class LLTest extends OpMode {
    DriveTrain driveTrain = new DriveTrain(DriveBackRight, DriveBackLeft, DriveFrontRight, DriveFrontLeft, telemetry, Imu,odometry);

    @Override
    protected void run() {
        while(opModeIsActive()){
            Pose2D poser;
            try {
                poser = getLLPose();
                if(poser!=null) {
                    turnToGoal(poser.getX(DistanceUnit.CM), poser.getY(DistanceUnit.CM));
                }
                telemetry.addData("llpose",poser);
            }
            catch (NullPointerException e){
                telemetry.addData("pose", "oops thats null!");
            }
            telemetry.addData("odo pose", odometry.getPosition());
            odometry.update();
            telemetry.update();

        }
    }
    @Override
    protected void postInit(){
        limelight.setPollRateHz(100);
        limelight.start();
        odometry.resetPosAndIMU();
    }
    @Override
    protected void end() {

    }

    private Pose2D getLLPose(){
        double heading = odometry.getHeading(AngleUnit.DEGREES);
        limelight.updateRobotOrientation(heading);
        LLResult curResult = limelight.getLatestResult();
        if(curResult != null && curResult.isValid()){
            Pose3D botPose = curResult.getBotpose_MT2();
            double x = curResult.getTx();
            double y = curResult.getTy();
            telemetry.addData("Tx", curResult.getTx());
            telemetry.addData("Ty", curResult.getTy());
            telemetry.addData("Ta", curResult.getTa());
            return new Pose2D(DistanceUnit.CM,x*10,y*10,AngleUnit.DEGREES,heading);
            // getposition returns mm and degrees (i think)
        }
        return null;
    }
    public void turnToGoal(double x, double y){
        double lenfield = 361; // cm
        double yOffset = 20;//prev = 18
        double xOffset = 20; // prev = 16
        x = lenfield/2 - x;
        double newY = lenfield/2 + y;

        double deg = Math.toDegrees(Math.atan((newY - yOffset)/(x - xOffset)));
        driveTrain.turnToGyro(-deg);
    }
}
