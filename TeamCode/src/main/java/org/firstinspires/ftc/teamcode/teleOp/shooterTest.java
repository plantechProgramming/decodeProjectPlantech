package org.firstinspires.ftc.teamcode.teleOp;

import com.acmerobotics.dashboard.config.Config;
import com.bylazar.configurables.annotations.Configurable;

import org.firstinspires.ftc.teamcode.OpMode;
import org.firstinspires.ftc.teamcode.teleOp.actions.Shooter;

@Configurable
@Config
@com.qualcomm.robotcore.eventloop.opmode.TeleOp
public class shooterTest extends OpMode {
    @Override
    protected void postInit() {
        Imu.resetYaw();
    }

    @Override
    protected void run() {
        Shooter shooter = new Shooter(shootMotor,dashboardTelemetry,shootMotorOp);
        double forward;
        while(opModeIsActive()){
            shooter.variableSpeedShoot(gamepad1.dpad_up, gamepad1.dpad_down, 0.05);
            dashboardTelemetry.addData("wanted", 3000);
            dashboardTelemetry.addData("pos2",shooter.shooter2.getCurrentPosition());
            dashboardTelemetry.addData("pos",shooter.shooter.getCurrentPosition());
            dashboardTelemetry.update();
        }
    }

    @Override
    protected void end() {

    }
}
