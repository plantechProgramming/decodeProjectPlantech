package org.firstinspires.ftc.teamcode.teleOp;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.Misc.Utils.Alliance;
import org.firstinspires.ftc.teamcode.Misc.Txt.ReadWrite;
import org.firstinspires.ftc.teamcode.Misc.Utils.Converters;
import org.firstinspires.ftc.teamcode.Misc.Utils.Extras;
import org.firstinspires.ftc.teamcode.subsystems.AutoCommands;
import org.firstinspires.ftc.teamcode.auto.pedro.Constants;
import org.firstinspires.ftc.teamcode.subsystems.DriveTrain;
import org.firstinspires.ftc.teamcode.TeamOpMode;

import org.firstinspires.ftc.teamcode.subsystems.InBetween;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;

@Config
@TeleOp
public class TeleOpRed extends TeamOpMode {
    Follower follower;
    @Override
    protected void postInit() {
        Alliance.set(Alliance.RED);
        follower = Constants.createFollower(hardwareMap);
    }

    @Override
    public void run(){
        Intake intake  = new Intake();
        InBetween inBetween = new InBetween();
        Shooter shooter = new Shooter();
        DriveTrain driveTrain = new DriveTrain();
        ReadWrite readWrite = new ReadWrite();
        ElapsedTime elapsedTime = new ElapsedTime();
        AutoCommands commands = new AutoCommands();


        double forward; //-1 to 1
        double turn;
        double drift;
        double botHeading;
        boolean shooting;
        boolean turning;
        boolean activatedHold = false;
        boolean holdInitialized = false;

        odometry.setPosition(Converters.PedroPoseConverter(readWrite.readPose()));
        odometry.update();
        Pose lastPos = follower.getPose();

        while (opModeIsActive() ) {
            elapsedTime.reset();

            forward = -gamepad1.left_stick_y;
            turn = gamepad1.right_stick_x;
            drift = gamepad1.left_stick_x;
            shooting = gamepad1.right_bumper;
            turning = gamepad1.left_bumper;

            botHeading = odometry.getHeading(AngleUnit.DEGREES);

            if(shooting){
                commands.shoot();
                if(!holdInitialized){
                    Extras.setDriveToFloatMode();
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
                    Extras.setDriveToBrakeMode();
                }

                if(turning){
                    if (gamepad1.right_trigger > 0){
                        commands.take();
                    }
                    else if(gamepad1.left_trigger!=0) {
                        commands.partialOut();
                    }
                    else if(gamepad1.x){
                        commands.out();
                    }
                    else{
                        commands.stopAll();
                    }
                }
                else{
                    driveTrain.drive(-forward, -drift, turn, botHeading, 1);//TODO: change for RED -forward, -drift
                }
            }

            shooter.interpolationVariableShoot(gamepad1.dpad_up, gamepad1.dpad_down, 0.01);


            if(gamepad1.back) {
                odometry.setPosition(new Pose2D(DistanceUnit.CM, 0, 0, AngleUnit.DEGREES, 180)); //TODO: change for RED heading 180
            }

            driveTrain.updateTelemetry(telemetry);
            driveTrain.updateTelemetry(dashboardTelemetry);

            shooter.updateTelemetry(telemetry);
            shooter.updateTelemetry(dashboardTelemetry);

            telemetry.update();
            dashboardTelemetry.update();
            commands.periodic();
            follower.update();
        }

    }

    @Override
    protected void end() {

    }
}
