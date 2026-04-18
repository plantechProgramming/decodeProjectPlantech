package org.firstinspires.ftc.teamcode.teleOp;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.OpMode;

@TeleOp
public class LLTest extends OpMode {

    @Override
    protected void run() {
        telemetry.setMsTransmissionInterval(11);
        ll.start();
        waitForStart();
//        shootMotor.setPower(0.6);
//        shootMotorOp.setPower(-0.6);
        while (opModeIsActive()) {
            LLResult result = ll.getLatestResult();
            if (result != null) {
                if (result.isValid()) {
                    Pose3D botpose = result.getBotpose();
//                    telemetry.addData("tx", result.getTx());
//                    telemetry.addData("ty", result.getTy());
                    dashboardTelemetry.addData("Botpose", botpose.toString());
                }
                else{
                    dashboardTelemetry.addLine("not valid");
                }
            }
            else{
                dashboardTelemetry.addLine("doesnt see anything :(((");
            }
            dashboardTelemetry.update();
        }
    }

    @Override
    protected void end() {

    }
}
