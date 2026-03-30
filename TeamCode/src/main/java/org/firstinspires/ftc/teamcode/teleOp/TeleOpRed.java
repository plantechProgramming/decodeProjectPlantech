package org.firstinspires.ftc.teamcode.teleOp;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.CameraName;
import org.firstinspires.ftc.robotcore.external.matrices.MatrixF;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Quaternion;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.robotcore.internal.network.ControlHubApChannelManager;
import org.firstinspires.ftc.teamcode.auto.autos.ReadWrite;
import org.firstinspires.ftc.teamcode.auto.camera.AprilTagLocalization;
import org.firstinspires.ftc.teamcode.auto.camera.aprilTagsTest;
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

@Config
@com.qualcomm.robotcore.eventloop.opmode.TeleOp
public class TeleOpRed extends OpMode {
    Follower follower;
    AprilTagLocalization tagLocalization;
    @Override
    protected void postInit() {
        follower = Constants.createFollower(hardwareMap);
        tagLocalization = new AprilTagLocalization("RED", telemetry); //TODO: change here for red
        tagLocalization.initProcessor(hardwareMap);
        while (tagLocalization.visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING){
            sleep(20);
        }
        tagLocalization.applySettings();
    }

    @Override
    public void run(){
        odometry.resetPosAndIMU();
        Intake intake  = new Intake(inBetweenMotor,shooterIBL,shooterIBR,intakeMotor,telemetry);
        DriveTrain driveTrain = new DriveTrain(DriveBackRight, DriveBackLeft, DriveFrontRight, DriveFrontLeft, telemetry, Imu,odometry, "RED");
        Shooter shooter = new Shooter(shootMotor,dashboardTelemetry,shootMotorOp, odometry);
        ReadWrite readWrite = new ReadWrite();
        Utils utils = new Utils(telemetry,odometry);
        //ColorSensorTest cSensor = new ColorSensorTest();
        GetVelocity shooterVel = new GetVelocity(shootMotor,0.1);


        //TODO: find why didnt work outside
        sleep(100);
        double forward; //-1 to 1
        double turn;
        double drift;
        double botHeading;
        boolean slow = false;
        boolean turretActivated = false;
        boolean activatedHold = false;
        boolean turningTowardsGoal = false;
        boolean aang = false;
        double tick = 2000/(48*Math.PI); //per tick
        Pose2D robotPoseFromCam = null;
        Pose2D filteredPose = null;
        int count = 200;
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
            forward = -gamepad1.left_stick_y;
            turn = gamepad1.right_stick_x;
            drift = gamepad1.left_stick_x;
            //todo: pinpoint
            botHeading = odometry.getHeading(AngleUnit.RADIANS);
//            shooter.interpolate(utils.getDistFromGoal("RED")); //TODO: change for RED
            shooter.variableInterplationSpeedShoot(gamepad1.dpad_up, gamepad1.dpad_down, 0.01, "RED"); //TODO: change for RED
            ElapsedTime elapsedTime = new ElapsedTime();
            if(!gamepad1.left_bumper && !gamepad1.right_bumper) {
                driveTrain.drive(-forward, -drift, turn, botHeading, 1);//TODO: change for RED -forward, -drift
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
                if(shooter.isUpToGivenSpeed(shooter.interpolateTel(utils.getDistFromGoal("RED")))){ //TODO: change for RED
                    intake.inBetweenInFull();
                }
                else{
                    intake.stopIntake();
                }
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
            if(!gamepad1.left_bumper){
                driveTrain.usingCamForTurn = false;
            }
            if(gamepad1.left_bumper && !gamepad1.right_bumper){
                driveTrain.turnToGyro(utils.getAngleFromGoal("RED"));// TODO: change for RED
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
            tagLocalization.detectTags();
            if(gamepad1.dpad_right && tagLocalization.goalTag != null){
               driveTrain.turnTowardsAprilTag(tagLocalization.goalTag);
            }
            if(gamepad1.dpad_left){
                tagLocalization.detectTags();
                driveTrain.turnToGoal("RED", tagLocalization.goalTag);
            }


            if(gamepad1.back) {
                odometry.setPosition(new Pose2D(DistanceUnit.CM, 0, 0, AngleUnit.DEGREES, 180)); //TODO: change for RED
            }
            if(driveTrain.isStopped()){
                if(tagLocalization.goalTag != null){
                    filteredPose = driveTrain.filterCamPose(tagLocalization.getRobotPose(tagLocalization.goalTag));
                    if((count >= 300 && !gamepad1.right_bumper && utils.PoseThreshold(filteredPose, odometry.getPosition(), 100, 10))){
                        odometry.setPosition(new Pose2D(DistanceUnit.CM, odometry.getPosX(DistanceUnit.CM), odometry.getPosY(DistanceUnit.CM), AngleUnit.DEGREES, filteredPose.getHeading(AngleUnit.DEGREES)));
                        sleep(150);
                        telemetry.addLine("SET POSITION");
                        count = 0;
                    }
                }
            }
            else{
//                driveTrain.lastFilteredPose = driveTrain.filteredPose;
                utils.xPos.clear();
                utils.yPos.clear();
                utils.headPos.clear();
            }
            count++;
            telemetry.addData("count", count);
            driveTrain.setDriveTelemetry(telemetry);
            driveTrain.setDriveTelemetry(dashboardTelemetry);
//
            shooter.setShooterTelemetry(telemetry);
            shooter.setShooterTelemetry(dashboardTelemetry);

            tagLocalization.setCameraTelemetry(telemetry);
            tagLocalization.setCameraTelemetry(dashboardTelemetry);

            telemetry.addData("wanted interpolation", shooter.interpolateTel(utils.getDistFromGoal("RED")) *6000);
            dashboardTelemetry.addData("wanted interpolation", shooter.interpolateTel(utils.getDistFromGoal("RED")) *6000);
            telemetry.addData("filtered cam pose", filteredPose);
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
