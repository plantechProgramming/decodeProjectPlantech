package org.firstinspires.ftc.teamcode.teleOp;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.ftc.FTCCoordinates;
import com.pedropathing.ftc.InvertedFTCCoordinates;
import com.pedropathing.ftc.PoseConverter;
import com.pedropathing.geometry.CoordinateSystem;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.CameraName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.auto.autos.ReadWrite;
import org.firstinspires.ftc.teamcode.auto.camera.AprilTagLocalization;
import org.firstinspires.ftc.teamcode.auto.camera.colorsensor.ColorSensorTest;
import org.firstinspires.ftc.teamcode.teleOp.actions.DriveTrain;
import org.firstinspires.ftc.teamcode.teleOp.actions.GetVelocity;
import org.firstinspires.ftc.teamcode.teleOp.actions.Intake;
import org.firstinspires.ftc.teamcode.OpMode;

import org.firstinspires.ftc.teamcode.teleOp.actions.Shooter;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagGameDatabase;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import dev.nextftc.core.commands.delays.Delay;

@Config
@com.qualcomm.robotcore.eventloop.opmode.TeleOp
public class TeleOpBlue extends OpMode {
    @Override
    protected void postInit() {
        //TODO: pinpoint
        Imu.resetYaw();
    }

    public final Position CAM_POS = new Position(DistanceUnit.CM, 0, 0, 0, 0);
    private VisionPortal visionPortal;
    private final YawPitchRollAngles CAM_ORIENTATION = new YawPitchRollAngles(AngleUnit.DEGREES,0,-90,0,0);

    @Override
    public void run(){
        odometry.resetPosAndIMU();
        Intake intake  = new Intake(intakeIBL,intakeIBR,shooterIBL,shooterIBR,intakeMotor,telemetry);
        DriveTrain driveTrain = new DriveTrain(DriveBackRight, DriveBackLeft, DriveFrontRight, DriveFrontLeft, telemetry, Imu,odometry);
        Shooter shooter = new Shooter(shootMotor,dashboardTelemetry,shootMotorOp);
        ReadWrite readWrite = new ReadWrite();
        //ColorSensorTest cSensor = new ColorSensorTest();
        GetVelocity shooterVel = new GetVelocity(shootMotor,0.1);


        //TODO: find why didnt work outside
        AprilTagLocalization test = new AprilTagLocalization("BLUE"); //TODO: change here for red
        AprilTagProcessor aprilTag = new AprilTagProcessor.Builder()
                .setCameraPose(CAM_POS, CAM_ORIENTATION)
                .setTagFamily(AprilTagProcessor.TagFamily.TAG_36h11)
                .setTagLibrary(AprilTagGameDatabase.getCurrentGameTagLibrary())
                .setOutputUnits(DistanceUnit.METER, AngleUnit.DEGREES)
                .build();

        VisionPortal.Builder builder = new VisionPortal.Builder();
        builder.setCamera(hardwareMap.get(CameraName.class,"webcam"));

        builder.addProcessor(aprilTag);
        visionPortal = builder.build();

//        AprilTagProcessor aprilTag = test.initAprilTag();
//
//        VisionPortal visionPortal = new VisionPortal.Builder()
//                .setCamera(hardwareMap.get(WebcamName.class, "webcam"))
//                .addProcessor(aprilTag)
//                .build();

        sleep(100);
        double forward; //-1 to 1
        double turn;
        double drift;
        double botHeading;
        boolean slow = false;
        double tick = 2000/(48*Math.PI); //per tick
        odometry.setPosition(driveTrain.PedroPoseConverter(readWrite.readPose()));

        while (opModeIsActive() ) {
            AprilTagDetection goalTag = test.specialDetection;
//            test.telemetryAprilTag(aprilTag);
            test.detectTags(aprilTag);
            forward = -gamepad1.left_stick_y;
            turn = gamepad1.right_stick_x;
            drift = gamepad1.left_stick_x;
            //todo: pinpoint
            botHeading = odometry.getHeading(AngleUnit.RADIANS);

            ElapsedTime elapsedTime = new ElapsedTime();
            if(!gamepad1.left_bumper) {
                driveTrain.drive(forward, drift, turn, botHeading, 1);//TODO: change for RED
            }

            if (gamepad1.right_trigger > 0){
                intake.intakeIn();
                intake.inBetweenInPart();
            }
            else if(gamepad1.left_trigger!=0) {
                shooter.out();
                intake.inBetweenOut();
                intake.intakeOut();
            }
            else if(gamepad1.x){
                intake.inBetweenOut();
                shooter.out();
//            } else if (gamepad1.a) {
//                intake.inBetweenInPart();
            }
//           if(gamepad1.dpad_up && test.specialDetection != null && test.numDetected > 0){
//               double deg = test.specialDetection.ftcPose.bearing;
//
//               driveTrain.turnToGyro(odometry.getHeading(AngleUnit.DEGREES) + deg);
//               telemetry.addData("yaw", deg);
//               telemetry.update();
//           }
            else if(gamepad1.right_bumper){
                if(shooter.isUpToSpeed()){
                    intake.inBetweenInFull();
                }
                intake.intakeIn();
            }

            else{
//                shooter.stopShooter();
                intake.stopIntake();
            }
//            shooter.variableSpeedShoot(gamepad1.y, gamepad1.a, .05);
            shooter.naiveShooter(driveTrain.isFar());

            if(gamepad1.left_bumper){
                driveTrain.turnToGoal("BLUE");//TODO: change for RED
            }
            if(gamepad1.start){
                double x = odometry.getPosX(DistanceUnit.CM);
                double y = odometry.getPosY(DistanceUnit.CM);
                Pose2D curPose = new Pose2D(DistanceUnit.CM,x,y,AngleUnit.DEGREES,0);//TODO: change for RED
                odometry.setPosition(curPose);
            }
            if(gamepad1.back){
                odometry.setPosition(new Pose2D(DistanceUnit.CM,0,0,AngleUnit.DEGREES, 0)); //TODO: change for RED

            }

            driveTrain.setDriveTelemetry(telemetry);
            driveTrain.setDriveTelemetry(dashboardTelemetry);

            shooter.setShooterTelemetry(telemetry);
            shooter.setShooterTelemetry(dashboardTelemetry);

            telemetry.update();
            dashboardTelemetry.update();
            odometry.update();
        }

    }


    @Override
    protected void end() {

    }
}
