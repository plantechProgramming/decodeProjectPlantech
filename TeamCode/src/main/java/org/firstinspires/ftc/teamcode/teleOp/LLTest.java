package org.firstinspires.ftc.teamcode.teleOp;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.OpMode;

@TeleOp
public class LLTest extends OpMode {

    private static int count = 0;
    @Override
    protected void run() {
        telemetry.setMsTransmissionInterval(11);
        ll.start();
        waitForStart();
        Limelight limeLight = new Limelight(ll);
        shootMotor.setPower(0.6);
        shootMotorOp.setPower(-0.6);
        while (opModeIsActive()) {
            LLResult result = ll.getLatestResult();
            try{
                limeLight.updateFilter();
                dashboardTelemetry.addData("raw heading", result.getBotpose().getOrientation().getYaw(AngleUnit.DEGREES));
                dashboardTelemetry.addData("raw heading", limeLight.getRawHeadingOdoCoords());
                count++;
                if(count > 100) {
                    dashboardTelemetry.addData("Heading", limeLight.getFilteredHeadingOdoCoords());
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
