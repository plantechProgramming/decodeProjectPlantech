package org.firstinspires.ftc.teamcode.teleOp;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.CameraName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.auto.camera.AprilTagLocalization;
import org.firstinspires.ftc.teamcode.auto.camera.colorsensor.ColorSensorTest;
import org.firstinspires.ftc.teamcode.teleOp.actions.DriveTrain;
import org.firstinspires.ftc.teamcode.teleOp.actions.Intake;
import org.firstinspires.ftc.teamcode.OpMode;

import org.firstinspires.ftc.teamcode.teleOp.actions.Shooter;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagGameDatabase;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

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
        Intake intake  = new Intake(intakeIBL,intakeIBR,shooterIBL,shooterIBR,intakeMotor,telemetry);
        DriveTrain driveTrain = new DriveTrain(DriveBackRight, DriveBackLeft, DriveFrontRight, DriveFrontLeft, telemetry, Imu,odometry);
        Shooter shooter = new Shooter(shootMotor,dashboardTelemetry,shootMotorOp);
        ColorSensorTest cSensor = new ColorSensorTest();
        cSensor.init(hardwareMap);

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

//        odometry.setPosition(new Pose2D(DistanceUnit.CM,-74,154,AngleUnit.DEGREES, 0));
        odometry.setPosition(new Pose2D(DistanceUnit.CM,90,-165,AngleUnit.DEGREES, 0));//TODO: change here for red


//        AprilTagProcessor aprilTag = test.initAprilTag();
//
//        VisionPortal visionPortal = new VisionPortal.Builder()
//                .setCamera(hardwareMap.get(WebcamName.class, "webcam"))
//                .addProcessor(aprilTag)
//                .build();


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
            //todo: pinpoint
            botHeading = odometry.getHeading(AngleUnit.RADIANS);

            ElapsedTime elapsedTime = new ElapsedTime();
            driveTrain.drive(forward, drift, turn, botHeading, 1);

            // todo: figure out if driver wants slowing on start
 /*           if(gamepad1.x && !slow){
                driveTrain.drive(forward, drift, turn, botHeading, 0.5);
                slow = true;
            }else if(gamepad1.x && slow) {
                driveTrain.drive(forward, drift, turn, botHeading, 1);
                slow = false;
            }
  */

            if (gamepad1.a){
                intake.intakeIn();
                intake.inBetweenInPart();
            }
            else if (gamepad1.b){
                intake.inBetweenInFull();
            }
            else if(gamepad1.x) {
                intake.intakeOut();
                intake.inBetweenOut();

                shooter.shooter.setPower(-0.2);
                shooter.shooter2.setPower(0.2);
            }
            else{
                intake.intake_motor.setPower(0);
                intake.ibl.setPower(0);
                intake.ibr.setPower(0);
                intake.sr.setPower(0);
                intake.sl.setPower(0);
            }

            if(gamepad1.dpad_right){
                driveTrain.turnToGyro(-160);
            }
            if(gamepad1.dpad_left){
                driveTrain.turnToGoal();
            }
           if(gamepad1.dpad_up && test.specialDetection != null && test.numDetected > 0){
               double deg = test.specialDetection.ftcPose.bearing;

               driveTrain.turnToGyro(odometry.getHeading(AngleUnit.DEGREES) + deg);
               telemetry.addData("yaw", deg);
               telemetry.update();
           }
//            if (gamepad1.left_bumper) {
//                shooter.naiveShooter(false);
//                dashboardTelemetry.addLine("close");
//                dashboardTelemetry.update();
//
//            }else if (gamepad1.right_bumper){
//                shooter.naiveShooter(true);
//                dashboardTelemetry.addLine("far");
//                dashboardTelemetry.update();}
            if(gamepad1.left_bumper){
                if (odometry.getPosY(DistanceUnit.CM) > 60){
                    shooter.naiveShooter(true);
                    dashboardTelemetry.addLine("far");
                    dashboardTelemetry.update();
                }
                else{
                    shooter.naiveShooter(false);
                    dashboardTelemetry.addLine("close");
                    dashboardTelemetry.update();
                }
            }

            else{
                shooter.stopShooter();
            }
//            else{
//                shooter.stopShooter();
//            }


            //intake.intakeTest(gamepad1.y);
            //TODO: make use pinpoint
            if(gamepad1.start){
                double x = odometry.getPosX(DistanceUnit.CM);
                double y = odometry.getPosY(DistanceUnit.CM);
                double heading = odometry.getHeading(AngleUnit.DEGREES);
                Pose2D curPose = new Pose2D(DistanceUnit.CM,x,y,AngleUnit.DEGREES,0);
                odometry.setPosition(curPose);
            }
            if(gamepad1.back){
                odometry.setPosition(new Pose2D(DistanceUnit.CM,0,0,AngleUnit.DEGREES, 0));

            }

            dashboardTelemetry.addData("botheading",odometry.getHeading(AngleUnit.DEGREES));
            dashboardTelemetry.addData("botheadingIMU",Imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES));
            dashboardTelemetry.addData("X pos: ", odometry.getPosX(DistanceUnit.CM));
            dashboardTelemetry.addData("Y pos: ", odometry.getPosY(DistanceUnit.CM));
//            telemetry.addData("heading", odometry.getHeading(AngleUnit.DEGREES));
            dashboardTelemetry.addData("X encoder", odometry.getEncoderX());
            dashboardTelemetry.addData("Y encoder", odometry.getEncoderY());
            /*dashboardTelemetry.addData("shooter power: ",shooter.shooter2.getVelocity(AngleUnit.DEGREES));
            dashboardTelemetry.addData("odometry blabla: ",odometry.getCurrentPosition()/tick);
            dashboardTelemetry.addData("last Detected Color: ", cSensor.getLastDetected());
            */dashboardTelemetry.update();
            odometry.update();

        }

    }

    @Override
    protected void end() {

    }
    }
