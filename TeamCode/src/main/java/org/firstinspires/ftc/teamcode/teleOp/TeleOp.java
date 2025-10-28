package org.firstinspires.ftc.teamcode.teleOp;

import com.acmerobotics.dashboard.config.Config;
import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.CameraName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.auto.camera.AprilTagLocalization;
import org.firstinspires.ftc.teamcode.auto.camera.colorsensor.ColorSensorTest;
import org.firstinspires.ftc.teamcode.teleOp.actions.Intake;
import org.firstinspires.ftc.teamcode.OpMode;
import org.firstinspires.ftc.teamcode.auto.camera.aprilTagsTest;

import org.firstinspires.ftc.teamcode.teleOp.actions.Shooter;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagGameDatabase;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;

import dev.nextftc.core.units.Distance;


@Configurable
@Config
@com.qualcomm.robotcore.eventloop.opmode.TeleOp
public class TeleOp extends OpMode {
    @Override
    protected void postInit() {
        Imu.resetYaw();
    }
    public final Position CAM_POS = new Position(DistanceUnit.CM,
            0, 0, 0, 0);
    private VisionPortal visionPortal;
    private final YawPitchRollAngles CAM_ORIENTATION = new YawPitchRollAngles(AngleUnit.DEGREES,0,-90,0,0);

    @Override
    public void run(){
//        DriveTrain driveTrain = new DriveTrain(DriveBackRight, DriveBackLeft, DriveFrontRight, DriveFrontLeft, telemetry, Imu);
        Shooter shooter = new Shooter(shootMotor,telemetry,shootMotorOp);
        ColorSensorTest cSensor = new ColorSensorTest();
        cSensor.init(hardwareMap);
        boolean is_up = false;
        AprilTagLocalization test = new AprilTagLocalization();
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

       /* VisionPortal visionPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "webcam"))
                .addProcessor(aprilTag)
                .build();
*/

        double forward; //-1 to 1
        double turn;
        double drift;
        double botHeading;
        boolean slow = false;
        double tick = 2000/(48*Math.PI); //per tick
        while (opModeIsActive() ) {
            AprilTagDetection goalTag = test.specialDetection;
//            test.telemetryAprilTag(aprilTag);
            test.detectTags(aprilTag);

            forward = -gamepad1.left_stick_y;
            turn = gamepad1.right_stick_x;
            drift = gamepad1.left_stick_x;
            botHeading = Imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

            ElapsedTime elapsedTime = new ElapsedTime();
//            driveTrain.drive(forward, drift, turn, botHeading, 1);
//            telemetry.addData("x", DriveFrontRight.getCurrentPosition());
//            telemetry.addData("y",DriveBackLeft.getCurrentPosition());

            if(gamepad1.x && !slow){
//                driveTrain.drive(forward, drift, turn, botHeading, 0.5);
                slow = true;
            }else if(gamepad1.x && slow) {
//                driveTrain.drive(forward, drift, turn, botHeading, 1);
                slow = false;
            }
//            } telemetry.addData("y: ", DriveBackLeft.getCurrentPosition());
//            telemetry.addData("x:", DriveFrontRight.getCurrentPosition());
            shooter.shooterTest(-gamepad1.left_stick_y);
            try {
                double d = test.distanceToGoal(goalTag.robotPose,goalTag.id);
//                double t = Math.abs(Math.sqrt((2*Math.tan(Math.toRadians(67))*d-1.9)/9.8));
//                double t = 2;
//                double d = 1;
//                telemetry.addData("time", t);
                if (gamepad1.y) shooter.shoot(d, 2);
                else{shooter.noPhysShoot(0);}
            }
            catch(NullPointerException e){
                telemetry.addLine("Npe triggered");
            }


            //intake.intakeTest(gamepad1.y);
            if(gamepad1.back){Imu.resetYaw();}

            telemetry.addData("recognized color: ", cSensor.getDetectedColor(telemetry));
            telemetry.addData("number of apriltags detected",test.numDetected);
            if(goalTag != null){
                telemetry.addData("distance from tag: ", test.distanceToGoal(goalTag.robotPose,goalTag.id));
//                telemetry.addData("distance from tag X: ", test.specialDetection.robotPose.getPosition().x);
            }
            else{
                telemetry.addData("distance from tag", "null :`(((");
            }
            telemetry.addData("Order: ",test.Order);

            telemetry.addData("theta", shooter.theta);
            telemetry.addData("shooter power: ",shooter.motorPower);
            telemetry.addData("odometry: ",odometry.getCurrentPosition()/tick);
            telemetry.update();
        }

    }

    @Override
    protected void end() {

    }

}
