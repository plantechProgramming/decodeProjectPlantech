package org.firstinspires.ftc.teamcode.Tests.TeleOp;

import static com.pedropathing.ivy.Scheduler.schedule;

import com.pedropathing.ivy.Scheduler;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.TeamOpMode;
import org.firstinspires.ftc.teamcode.subsystems.AutoCommands;
import org.firstinspires.ftc.teamcode.subsystems.DriveTrain;

@TeleOp(group = "teleOp tests")
public class DriveTest extends TeamOpMode {

    @Override
    protected void postInit(){
        odometry.resetPosAndIMU();
        sleep(250);
    }
    @Override
    protected void run() {

        DriveTrain driveTrain = new DriveTrain();

        double gamepadForward; //-1 to 1
        double gamepadTurn;
        double gamepadDrift;
        double botHeading;

        while(opModeIsActive()){
            gamepadForward = -gamepad1.left_stick_y;
            gamepadTurn = gamepad1.right_stick_x;
            gamepadDrift = gamepad1.left_stick_x;

            botHeading = odometry.getHeading(AngleUnit.DEGREES);
            schedule(driveTrain.drive(-gamepadForward, -gamepadDrift, gamepadTurn, botHeading, 1));//TODO: change for RED -forward, -drift

            driveTrain.updateTelemetry(telemetry);
            driveTrain.updateTelemetry(dashboardTelemetry);

            telemetry.update();
            odometry.update();
            dashboardTelemetry.update();
            Scheduler.execute();
        }
    }

    @Override
    protected void end() {

    }
}
