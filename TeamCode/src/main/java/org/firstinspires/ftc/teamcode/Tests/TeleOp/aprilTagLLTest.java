package org.firstinspires.ftc.teamcode.Tests.TeleOp;

import static com.pedropathing.ivy.Scheduler.schedule;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.pedropathing.follower.Follower;
import com.pedropathing.ivy.Scheduler;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes.ColorResult;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.Misc.Utils.Alliance;
import org.firstinspires.ftc.teamcode.Misc.Utils.DashboardCanvas;
import org.firstinspires.ftc.teamcode.Misc.Utils.PoseFunctions;
import org.firstinspires.ftc.teamcode.Misc.Utils.TelemetryUtils;
import org.firstinspires.ftc.teamcode.Misc.pedro.Constants;
import org.firstinspires.ftc.teamcode.TeamOpMode;
import org.firstinspires.ftc.teamcode.subsystems.AutoCommands;
import org.firstinspires.ftc.teamcode.subsystems.Camera.Limelight;

import java.util.List;

@Config
@TeleOp(group = "teleop tests")
public class aprilTagLLTest extends TeamOpMode {
    @Override
    public void postInit(){
        Alliance.set(Alliance.RED);
        Follower follower = Constants.createFollower(hardwareMap); // this line and the next line initialises the drivetrain motors
        follower.update();
        odometry.resetPosAndIMU();
    }

    @Override
    protected void run() {
//        AutoCommands commands = new AutoCommands();
        Limelight limelight = new Limelight();
        ll.start();
        Pose3D latestLLPos = null;
        Pose2D latestFilteredLLPos = null;
        while (opModeIsActive()){
//            commands.shooter.variableShoot(gamepad1.dpad_up, gamepad1.dpad_down, 0.005);
            try{
//                latestLLPos = limelight.getLatestBotpose();
//                latestFilteredLLPos = limelight.getFilteredBotPose();
//                telemetry.addData("XY field coords(moved)", limelight.getAbsoluteDetectionResult(odometry.getPosition()));
//                telemetry.addData("XY field coords(not moved)", limelight.getRotatedDetectionResult(odometry.getHeading(AngleUnit.DEGREES)));
//                telemetry.addData("XY robot coords", limelight.getDetectionResultAsPose2D());
                telemetry.addData("odo heading", odometry.getHeading(AngleUnit.DEGREES));
                telemetry.addData("odo x", odometry.getPosX(DistanceUnit.CM));
                telemetry.addData("odo y", odometry.getPosY(DistanceUnit.CM));
//                new DashboardCan vas()
//                        .addRobotAsCircle(latestFilteredLLPos)
//                        .addRobotAsCircle(PoseFunctions.pose3DToPose2D(latestLLPos, AngleUnit.DEGREES), "#00FF00")
//                        .draw();
            }
            catch (NullPointerException e){
//                telemetry.addLine("No apriltag found");
            }
//            telemetry.addData("robotPose", latestLLPos);
            telemetry.update();
//            schedule(commands.periodic());
            Scheduler.execute();
            odometry.update();

        }
    }

    @Override
    protected void end() {

    }
}
