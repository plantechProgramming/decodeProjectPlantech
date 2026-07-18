package org.firstinspires.ftc.teamcode.teleOp;

import static com.pedropathing.ivy.Scheduler.schedule;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.follower.Follower;
import com.pedropathing.ftc.InvertedFTCCoordinates;
import com.pedropathing.ftc.PoseConverter;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.ivy.Scheduler;
import com.pedropathing.paths.Path;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.Misc.DataSaving;
import org.firstinspires.ftc.teamcode.Misc.RobotPose;
import org.firstinspires.ftc.teamcode.Misc.Utils.Alliance;
import org.firstinspires.ftc.teamcode.Misc.Utils.Converters;
import org.firstinspires.ftc.teamcode.Misc.Utils.Extras;
import org.firstinspires.ftc.teamcode.Misc.Utils.PoseFunctions;
import org.firstinspires.ftc.teamcode.subsystems.AutoCommands;
import org.firstinspires.ftc.teamcode.auto.pedro.Constants;
import org.firstinspires.ftc.teamcode.subsystems.DriveTrain;
import org.firstinspires.ftc.teamcode.TeamOpMode;

import org.firstinspires.ftc.teamcode.subsystems.InBetween;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;

@Config
@TeleOp(group = "teleOp")
public class TeleOpRed extends TeamOpMode {
    Follower follower;

    @Override
    protected void postInit() {
        Alliance.set(Alliance.RED);
        follower = Constants.createFollower(hardwareMap);
    }

    @Override
    public void run(){
        DriveTrain driveTrain = new DriveTrain();
        ElapsedTime elapsedTime = new ElapsedTime();
        AutoCommands commands = new AutoCommands();
        PoseFunctions poseFunctions = new PoseFunctions(new RobotPose(odometry));

        double gamepadForward; //-1 to 1
        double gamepadTurn;
        double gamepadDrift;
        double botHeading;
        boolean shooting;
        boolean turning;
        boolean activatedHold = false;
        boolean holdInitialized = false;

        odometry.setPosition(PoseConverter.poseToPose2D(DataSaving.getEndPos(), InvertedFTCCoordinates.INSTANCE));
        odometry.update();
        Pose lastPos = follower.getPose();
        DriveTrain.setDriveToBrakeMode();

        while (opModeIsActive() ) {
            elapsedTime.reset();

            gamepadForward = -gamepad1.left_stick_y;
            gamepadTurn = gamepad1.right_stick_x;
            gamepadDrift = gamepad1.left_stick_x;
            shooting = gamepad1.right_bumper;
            turning = gamepad1.left_bumper;

            botHeading = odometry.getHeading(AngleUnit.DEGREES);

            if(shooting){
                schedule(commands.shoot());
                if(!holdInitialized){
                    DriveTrain.setDriveToFloatMode();
                    holdInitialized = true;
                    lastPos = follower.getPose();

                }
                follower.holdPoint(lastPos, false);
                activatedHold = true;
            }
            else {
                holdInitialized = false;
                if(activatedHold){
                    activatedHold = false;
                    follower.followPath(new Path(new BezierLine(follower.getPose(), follower.getPose())), false);
                    DriveTrain.setDriveToBrakeMode();
                }
                if(turning){
                    schedule(driveTrain.turnToAngle(poseFunctions.getAngleFromGoal(), botHeading));
                }
                else{
                    schedule(driveTrain.drive(-gamepadForward, -gamepadDrift, gamepadTurn, botHeading+90, 1));//TODO: change for RED +90
                }

                if (gamepad1.right_trigger > 0){
                    schedule(commands.take());
                }
                else if(gamepad1.left_trigger!=0) {
                    schedule(commands.partialOut());
                }
                else if(gamepad1.x){
                    schedule(commands.out());
                }
                else {
                    schedule(commands.stopAll());
                }
            }

            commands.shooter.interpolationVariableShoot(gamepad1.dpad_up, gamepad1.dpad_down, 0.01);


            if(gamepad1.back) {
                odometry.setPosition(new Pose2D(DistanceUnit.CM, 0, 0, AngleUnit.DEGREES, 0));
            }

            driveTrain.updateTelemetry(telemetry);
            driveTrain.updateTelemetry(dashboardTelemetry);

            commands.shooter.updateTelemetry(telemetry);
            commands.shooter.updateTelemetry(dashboardTelemetry);

            poseFunctions.updateTelemetry(telemetry);
            poseFunctions.updateTelemetry(dashboardTelemetry);
            telemetry.addData("endPose", DataSaving.getEndPos());

            telemetry.update();
            dashboardTelemetry.update();
            schedule(commands.periodic());
            follower.update();
            Scheduler.execute();
        }

    }

    @Override
    protected void end() {

    }
}
