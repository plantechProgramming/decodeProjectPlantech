package org.firstinspires.ftc.teamcode.teleOp;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.OpMode;
import org.firstinspires.ftc.teamcode.teleOp.actions.Shooter;

@TeleOp
public class LLTest extends OpMode {

    private static int count = 0;
    @Override
    protected void run() {
        telemetry.setMsTransmissionInterval(11);
        ll.start();
        waitForStart();
        Limelight limeLight = new Limelight(ll);
        Shooter shooter = new Shooter(shootMotor,telemetry,shootMotorOp,odometry);
        while (opModeIsActive()) {
            LLResult result = ll.getLatestResult();
            shooter.noPhysShootHomeostasis(0.6);
            try{
                limeLight.updateFilter();
//                dashboardTelemetry.addData("raw heading", result.getBotpose().getOrientation().getYaw(AngleUnit.DEGREES));
                dashboardTelemetry.addData("raw heading", limeLight.getRawHeadingLLCoords());
//                dashboardTelemetry.addData("filtered signed diffs", limeLight.utils.getFilteredSignedDiff());
                count++;
                if(count > 100) {
                    dashboardTelemetry.addData("Heading", limeLight.getFilteredHeadingLLCoords());
                    count = 0;
                }
            }
            catch (NullPointerException e){
                continue;
            }


//            if (result != null) {
//                if (result.isValid()) {

//                    dashboardTelemetry.addData("Heading", limeLight.getFilteredHeadingOdoCoords());
//                    Pose3D botpose = result.getBotpose();
//                    telemetry.addData("tx", result.getTx());
//                    telemetry.addData("ty", result.getTy());
//                    dashboardTelemetry.addData("Botpose", botpose.toString());
//                }
//                else{
//                    dashboardTelemetry.addLine("not valid");
//                }
//            }
//            else{
//                dashboardTelemetry.addLine("doesnt see anything :(((");
//            }
            dashboardTelemetry.update();
        }
    }

    @Override
    protected void end() {

    }
}
