package org.firstinspires.ftc.teamcode.teleOp;

import com.acmerobotics.dashboard.config.Config;
import com.bylazar.configurables.annotations.Configurable;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.OpMode;
import org.firstinspires.ftc.teamcode.teleOp.actions.Shooter;

@Configurable
@Config
@com.qualcomm.robotcore.eventloop.opmode.TeleOp
public class shooterTest extends OpMode {
    @Override
    protected void postInit() {
        odometry.resetPosAndIMU();
    }

    @Override
    protected void run() {
        while(opModeIsActive()){
            odometry.update();
            dashboardTelemetry.addData("raw posY",odometry.getEncoderY());
            dashboardTelemetry.addData("raw posX",odometry.getEncoderX());
            dashboardTelemetry.addData("posX", odometry.getPosX(DistanceUnit.CM));
            dashboardTelemetry.addData("posY", odometry.getPosY(DistanceUnit.CM));
            dashboardTelemetry.addData("imu", odometry.getHeading(AngleUnit.DEGREES));
            dashboardTelemetry.addData("status", odometry.getDeviceStatus());
            dashboardTelemetry.update();
        }
    }

    @Override
    protected void end() {

    }
}
