package org.firstinspires.ftc.teamcode.teleOp;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.follower.Follower;
import com.pedropathing.ftc.FTCCoordinates;
import com.pedropathing.ftc.InvertedFTCCoordinates;
import com.pedropathing.ftc.PoseConverter;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.CoordinateSystem;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.CameraName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.auto.autos.ReadWrite;
import org.firstinspires.ftc.teamcode.auto.camera.AprilTagLocalization;
import org.firstinspires.ftc.teamcode.auto.camera.colorsensor.ColorSensorTest;
import org.firstinspires.ftc.teamcode.auto.pedro.constants.Constants;
import org.firstinspires.ftc.teamcode.teleOp.actions.DriveTrain;
import org.firstinspires.ftc.teamcode.teleOp.actions.GetVelocity;
import org.firstinspires.ftc.teamcode.teleOp.actions.Intake;
import org.firstinspires.ftc.teamcode.OpMode;

import org.firstinspires.ftc.teamcode.teleOp.actions.Shooter;
import org.firstinspires.ftc.teamcode.teleOp.actions.Turret;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagGameDatabase;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.ArrayList;
import java.util.Arrays;

import dev.nextftc.core.commands.delays.Delay;
import kotlin.contracts.HoldsIn;

@Config
@com.qualcomm.robotcore.eventloop.opmode.TeleOp
public class TeleOpBlue extends OpMode {
    Follower follower;
//    AprilTagLocalization tagLocalization;
    Limelight limeLight;
    String team = "BLUE"; //TODO: change for BLUE
    @Override
    protected void postInit() {
//        odometry.recalibrateIMU();
        follower = Constants.createFollower(hardwareMap);
//        tagLocalization = new AprilTagLocalization(team, telemetry); //TODO: change here for red
//        limeLight = new Limelight(ll);
//        tagLocalization.initProcessor(hardwareMap);

        // while camera is not awake, sleep
//        while (tagLocalization.visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING){
//            sleep(20);
//        }
//        tagLocalization.applySettings();
//        odometry.resetPosAndIMU();
    }

    @Override
    public void run(){
        Intake intake  = new Intake(inBetweenMotor,shooterIBL,shooterIBR,intakeMotor,telemetry);
        DriveTrain driveTrain = new DriveTrain(DriveBackRight, DriveBackLeft, DriveFrontRight, DriveFrontLeft, telemetry, Imu,odometry, team,voltageSensor);
        Shooter shooter = new Shooter(shootMotor,dashboardTelemetry,shootMotorOp, odometry,voltageSensor);
        ReadWrite readWrite = new ReadWrite();
        Utils utils = new Utils(telemetry,odometry);
        ElapsedTime elapsedTime = new ElapsedTime();

        double forward; //-1 to 1
        double turn;
        double drift;
        double botHeading;
        boolean activatedHold = false;
        boolean aang = false;
        int count = 0;
        odometry.setPosition(utils.PedroPoseConverter(readWrite.readPose()));
        odometry.update();
        Pose lastPos = follower.getPose();
        DriveBackLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        DriveBackRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        DriveFrontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        DriveFrontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        limeLight.start();

        while (opModeIsActive() ) {
            elapsedTime.reset();
            forward = -gamepad1.left_stick_y;
            turn = gamepad1.right_stick_x;
            drift = gamepad1.left_stick_x;
            botHeading = odometry.getHeading(AngleUnit.RADIANS);
            if(!gamepad1.x){
                shooter.variableInterplationSpeedShoot(gamepad1.dpad_up, gamepad1.dpad_down, 0.01, team);
//                shooter.shooter.setPower(0.5);
//                shooter.shooter2.setPower(-0.5);
            }
//            shooter.noPhysShootHomeostasis(0.2);

            if(!gamepad1.left_bumper && !gamepad1.right_bumper) {
                driveTrain.drive(forward, drift, turn, botHeading, 1);//TODO: change for RED -forward, -drift
            }

            if(!gamepad1.right_bumper){
                lastPos = follower.getPose();
                aang = true;
                if(activatedHold){
                    activatedHold = false;
                    follower.followPath(new Path(new BezierLine(follower.getPose(), follower.getPose())), false);
                    DriveBackLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                    DriveBackRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                    DriveFrontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                    DriveFrontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                }

            }

            if (gamepad1.right_trigger > 0){
                intake.intakeIn();
                intake.inBetweenInPart();
            }
            else if(gamepad1.left_trigger!=0) {
                intake.inBetweenOut();
                intake.intakeOut();
            }
            else if(gamepad1.x){
                intake.inBetweenOut();
                intake.intakeOut();
                shooter.out();
            }
//            else if (gamepad1.dpad_right) {
//                intake.inBetweenInFull();
//            }else if (gamepad1.dpad_left){
//                intake.intake_motor.setPower(0.5);
//            }
            else if(gamepad1.right_bumper){
                if(shooter.isUpToGivenSpeed(shooter.getVariableInterplationSpeedShoot(false, false, 0, team))){
                    intake.inBetweenInFull();

                }
                else{
                    intake.inBetweenInFullSlow();
                }
//                else{
//                    intake.inBetweenInPart();
//                }
                intake.intakeIn();
                if(aang){
                    DriveBackLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
                    DriveBackRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
                    DriveFrontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
                    DriveFrontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
                    aang = false;
                    lastPos = follower.getPose();

                }
                follower.holdPoint(lastPos, false);
                activatedHold = true;
            }
            else{
                intake.stopIntake();
            }

            if(gamepad1.left_bumper && !gamepad1.right_bumper){
                driveTrain.turnToGyro(utils.getAngleFromGoal(team));

            }

            if(gamepad1.start) {
                odometry.setPosition(new Pose2D(DistanceUnit.CM, 0, 0, AngleUnit.DEGREES, 0)); //TODO: change for RED
            }
            if(gamepad1.back){
                odometry.setPosition(new Pose2D(DistanceUnit.CM, -158, 157, AngleUnit.DEGREES, 0));
            }
//            if(forward == 0 && drift == 0 && turn == 0 && utils.getDistFromGoal(team) < 260){
//                try{
//                    if(!limeLight.ll.isRunning()) limeLight.start(); // this try is called every loop,
//                    limeLight.updateFilter();
////                    dashboardTelemetry.addData("raw heading", limeLight.getRawHeadingLLCoords());
//                    count++;
////                    dashboardTelemetry.addData("Heading", limeLight.getFilteredHeadingLLCoords());
////                    dashboardTelemetry.addData("odo heading", limeLight.getFilteredHeadingOdoCoords());
//                    if(count > 50) {
//                        odometry.setPosition(new Pose2D(DistanceUnit.CM, odometry.getPosX(DistanceUnit.CM), odometry.getPosY(DistanceUnit.CM), AngleUnit.DEGREES, limeLight.getFilteredHeadingOdoCoords()));
//                        count = 0;
//                    }
//                }
//                catch (NullPointerException e){
////                    telemetry.addLine("no tag detected");
//                }
//            }
//            else{
//                count = 0;
//                limeLight.utils.prevFiltered = odometry.getHeading(AngleUnit.DEGREES);
//                limeLight.stop();
////                telemetry.addLine("started moving");
////                tagLocalization.filteredYawPrev = odometry.getHeading(AngleUnit.DEGREES);
//            }
//            telemetry.addData("count", count);
//            dashboardTelemetry.addData("count", count);
//            sleep(100);
            driveTrain.setDriveTelemetry(telemetry);
            driveTrain.setDriveTelemetry(dashboardTelemetry);
//            telemetry.addData("loop time", elapsedTime.milliseconds());
//            updateHisto(elapsedTime.milliseconds());
//            shooter.setShooterTelemetry(telemetry);
//            shooter.setShooterTelemetry(dashboardTelemetry);
//
//            dashboardTelemetry.addData("wanted interpolation", shooter.interpolateTel(utils.getDistFromGoal(team)) *6000);
//            telemetry.addData("wanted interpolation", shooter.interpolateTel(utils.getDistFromGoal(team)) *6000);
//            dashboardTelemetry.addData("pedro pose", follower.getPose());
//            telemetry.addData("pedro pose", follower.getPose());
//            telemetry.addData("time",elapsedTime.milliseconds());
//
//
            telemetry.update();
            dashboardTelemetry.update();
            follower.update();
//            utils.updateGoal();
        }

    }

    @Override
    protected void end() {
//        limeLight.shutDown();
//        telemetry.addData("histo", Arrays.toString(getHisto()));
//        telemetry.update();
//        System.out.println(Arrays.toString(getHisto()));
//        sleep(10000);
    }
    int[] arr = new int[5];
    public void updateHisto(double loopTime){
        if(0 <= loopTime && loopTime <= 0.5){
            arr[0]++;
        }
        else if(0.5 < loopTime && loopTime <= 1){
            arr[1]++;
        }
        else if(1 < loopTime && loopTime <= 10){
            arr[2]++;
        }
        else if(10 < loopTime && loopTime <= 20){
            arr[3]++;
        }
        else{
            arr[4]++;
        }

    }

    public int[] getHisto(){
        return arr;
    }
}
