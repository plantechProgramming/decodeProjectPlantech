package org.firstinspires.ftc.teamcode.teleOp;

import static com.pedropathing.ivy.groups.Groups.sequential;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.auto.autos.ReadWrite;
import org.firstinspires.ftc.teamcode.auto.camera.AprilTagLocalization;
import org.firstinspires.ftc.teamcode.auto.pedro.constants.Constants;
import org.firstinspires.ftc.teamcode.auto.subsystems.NextInBetween;
import org.firstinspires.ftc.teamcode.teleOp.actions.DriveTrain;
import org.firstinspires.ftc.teamcode.teleOp.actions.Intake;
import org.firstinspires.ftc.teamcode.OpMode;

import org.firstinspires.ftc.teamcode.teleOp.actions.Shooter;
import org.firstinspires.ftc.vision.VisionPortal;

@Config
@com.qualcomm.robotcore.eventloop.opmode.TeleOp
public class TeleOpBlue extends OpMode {
    Follower follower;
    AprilTagLocalization tagLocalization;
    @Override
    protected void postInit() {
        odometry.recalibrateIMU();
        follower = Constants.createFollower(hardwareMap);
        tagLocalization = new AprilTagLocalization("BLUE", telemetry); //TODO: change here for red
        tagLocalization.initProcessor(hardwareMap);

        // while camera is not awake, sleep
        while (tagLocalization.visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING){
            sleep(20);
        }
        tagLocalization.applySettings();
        odometry.resetPosAndIMU();
    }

    @Override
    public void run(){
        Intake intake  = new Intake(inBetweenMotor,shooterIBL,shooterIBR,intakeMotor,telemetry);
        DriveTrain driveTrain = new DriveTrain(DriveBackRight, DriveBackLeft, DriveFrontRight, DriveFrontLeft, telemetry, Imu,odometry, "RED");
        Shooter shooter = new Shooter(shootMotor,dashboardTelemetry,shootMotorOp, odometry);
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
        int stopCount = 0;
        String team = "BLUE"; //TODO: change for BLUE
//        follower.setStartingPose(readWrite.readPose());
        follower.update();
        odometry.setPosition(driveTrain.PedroPoseConverter(readWrite.readPose()));
        odometry.update();
        Pose lastPos = follower.getPose();
        DriveBackLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        DriveBackRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        DriveFrontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        DriveFrontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        while (opModeIsActive() ) {
            elapsedTime.reset();
            forward = -gamepad1.left_stick_y;
            turn = gamepad1.right_stick_x;
            drift = gamepad1.left_stick_x;
            botHeading = odometry.getHeading(AngleUnit.RADIANS);

            shooter.variableInterplationSpeedShoot(gamepad1.dpad_up, gamepad1.dpad_down, 0.01, team);

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
//                if(shooter.isUpToGivenSpeed(shooter.interpolateTel(utils.getDistFromGoal(team)))){
                intake.inBetweenInFull();
//                }
//                else{
//                    intake.stopPrimers();
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
//                turningTowardsGoal = true;
//                if(goalTag != null){
//                    if(goalTag.ftcPose.bearing < 0.5){
//                        turningTowardsGoal = false;
//                    }
//                }
//                if(Math.abs(utils.getAngleFromGoal("RED") - odometry.getHeading(AngleUnit.DEGREES)) < 0.5){
//                    turningTowardsGoal = false;
//                }
            }
//            tagLocalization.detectTags();
//            if(gamepad1.dpad_right && tagLocalization.goalTag != null){
//               driveTrain.turnTowardsAprilTag(tagLocalization.goalTag);
//            }
//            if(gamepad1.dpad_left){
//                tagLocalization.detectTags();
//                driveTrain.turnToGoal("RED", tagLocalization.goalTag);
//            }
//            if(!gamepad1.left_bumper){
//                driveTrain.usingCamForTurn = false;
//            }
            if(gamepad1.back) {
                odometry.setPosition(new Pose2D(DistanceUnit.CM, 0, 0, AngleUnit.DEGREES, 0)); //TODO: change for RED
            }
            if(forward == 0 && drift == 0 && turn == 0){
                stopCount++;
                tagLocalization.detectTags();
                if(tagLocalization.goalTag != null && utils.getDistFromGoal(team) < 220){
                    count++;
                    tagLocalization.getCurrDeg(tagLocalization.goalTag);
                    if(count >= 150 && !gamepad1.right_bumper){
                        double curHeading = tagLocalization.getCurrDeg(tagLocalization.goalTag);
                        odometry.setPosition(new Pose2D(DistanceUnit.CM, odometry.getPosX(DistanceUnit.CM), odometry.getPosY(DistanceUnit.CM), AngleUnit.DEGREES, curHeading));
                        telemetry.addLine("SET POSITION");
                        count = 0;
                    }
                }
            }
            else{
                count = 0;
                stopCount = 0;
                tagLocalization.filteredYawPrev = odometry.getHeading(AngleUnit.DEGREES);
            }

            telemetry.addData("count", count);
            dashboardTelemetry.addData("count", count);
            driveTrain.setDriveTelemetry(telemetry);
            driveTrain.setDriveTelemetry(dashboardTelemetry);
//
            shooter.setShooterTelemetry(telemetry);
            shooter.setShooterTelemetry(dashboardTelemetry);
//
            tagLocalization.setCameraTelemetry(telemetry);
            tagLocalization.setCameraTelemetry(dashboardTelemetry);
//
            telemetry.addData("wanted interpolation", shooter.interpolateTel(utils.getDistFromGoal(team)) *6000);
            dashboardTelemetry.addData("wanted interpolation", shooter.interpolateTel(utils.getDistFromGoal(team)) *6000);
//            telemetry.addData("time",elapsedTime.milliseconds());
//            telemetry.addData("stop count",stopCount);
            telemetry.update();
            dashboardTelemetry.update();
            odometry.update();
            follower.update();
        }

    }


    @Override
    protected void end() {

    }
}
