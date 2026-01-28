package org.firstinspires.ftc.teamcode.teleOp;

import com.qualcomm.hardware.limelightvision.LLResult;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.OpMode;

public class LLTest extends OpMode {
    @Override
    protected void run() {
        while(opModeIsActive()){
            try {
                telemetry.addData("pose",getLLPose(odometry.getHeading(AngleUnit.DEGREES)));
            }
            catch (NullPointerException e){
                telemetry.addData("pose", "oops thats null!");
            }
            telemetry.addData("odo pose", odometry.getPosition());
        }
    }
    @Override
    protected void postInit(){
        limelight.setPollRateHz(100);
        limelight.start();
    }
    @Override
    protected void end() {

    }

    private Pose2D getLLPose(double heading){
        limelight.updateRobotOrientation(heading);
        LLResult curResult = limelight.getLatestResult();
        double TaThresh = 30; // in percent?

        if(curResult != null && curResult.isValid() && curResult.getTa() > TaThresh){
            Pose3D botPose = curResult.getBotpose_MT2();
            double x = botPose.getPosition().x;
            double y = botPose.getPosition().y;

            // getposition returns mm and degrees (i think)
            return new Pose2D(DistanceUnit.MM,x, y, AngleUnit.DEGREES,heading);
        }
        return null;
    }
}
