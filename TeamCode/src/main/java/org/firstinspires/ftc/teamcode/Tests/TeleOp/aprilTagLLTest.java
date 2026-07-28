package org.firstinspires.ftc.teamcode.Tests.TeleOp;

import static com.pedropathing.ivy.Scheduler.schedule;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.pedropathing.ivy.Scheduler;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.Misc.Utils.Alliance;
import org.firstinspires.ftc.teamcode.Misc.Utils.PoseFunctions;
import org.firstinspires.ftc.teamcode.Misc.Utils.TelemetryUtils;
import org.firstinspires.ftc.teamcode.TeamOpMode;
import org.firstinspires.ftc.teamcode.subsystems.AutoCommands;
import org.firstinspires.ftc.teamcode.subsystems.Camera.Limelight;

@Config
@TeleOp(group = "teleop tests")
public class aprilTagLLTest extends TeamOpMode {
    @Override
    public void postInit(){
        Alliance.set(Alliance.RED);
    }

    @Override
    protected void run() {
        AutoCommands commands = new AutoCommands();
        Limelight limelight = new Limelight();
        ll.start();
        Pose3D latestLLPos = new Pose3D(new Position(DistanceUnit.CM, 0,0 ,0, 0), new YawPitchRollAngles(AngleUnit.DEGREES, 0, 0, 0, 0));
        while (opModeIsActive()){
            commands.shooter.variableShoot(gamepad1.dpad_up, gamepad1.dpad_down, 0.005);
            try{
                latestLLPos = limelight.getLatestBotpose();
            }
            catch (NullPointerException e){
                telemetry.addLine("No apriltag found");
            }
            TelemetryUtils.drawRobotAsCircle(PoseFunctions.pose3DToPose2D(latestLLPos, AngleUnit.DEGREES));
            telemetry.addData("robotPose", latestLLPos);
            telemetry.update();
            schedule(commands.periodic());
            Scheduler.execute();

        }
    }

    @Override
    protected void end() {

    }
}
