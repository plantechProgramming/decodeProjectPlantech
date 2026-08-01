package org.firstinspires.ftc.teamcode.Tests.TeleOp;

import static com.pedropathing.ivy.Scheduler.schedule;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.pedropathing.ivy.Scheduler;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.Misc.Utils.DashboardCanvas;
import org.firstinspires.ftc.teamcode.Misc.Utils.PoseFunctions;
import org.firstinspires.ftc.teamcode.TeamOpMode;
import org.firstinspires.ftc.teamcode.Misc.pedro.Constants;
import org.firstinspires.ftc.teamcode.subsystems.DriveTrain;

@TeleOp(group = "teleop tests")
public class DriveTest extends TeamOpMode {

    @Override
    protected void postInit(){
        Follower follower = Constants.createFollower(hardwareMap); // this line and the next line initialises the drivetrain motors
        follower.update();
        odometry.resetPosAndIMU();
        sleep(250);
    }
    @Override
    protected void run() {

        DriveTrain driveTrain = new DriveTrain();
        DriveTrain.setDriveToBrakeMode();

        double gamepadForward; //-1 to 1
        double gamepadTurn;
        double gamepadDrift;
        double botHeading;

        while(opModeIsActive()){
            gamepadForward = -gamepad1.left_stick_y;
            gamepadTurn = gamepad1.right_stick_x;
            gamepadDrift = gamepad1.left_stick_x;

            botHeading = odometry.getHeading(AngleUnit.DEGREES);
            schedule(driveTrain.drive(gamepadForward, gamepadDrift, gamepadTurn, botHeading+90, 1));//TODO: change for RED  -90
            driveTrain.updateTelemetry(telemetry);
            new DashboardCanvas()
                    .addRobotAsCircle(odometry.getPosition())
                    .draw();
            telemetry.update();
            Scheduler.execute();
            odometry.update();
        }
    }

    @Override
    protected void end() {

    }
}
