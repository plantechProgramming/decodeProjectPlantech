package org.firstinspires.ftc.teamcode.teleOp;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.auto.camera.AprilTagLocalization;
import org.firstinspires.ftc.teamcode.auto.camera.colorsensor.ColorSensorTest;
import org.firstinspires.ftc.teamcode.teleOp.actions.Intake;
import org.firstinspires.ftc.teamcode.OpMode;
import org.firstinspires.ftc.teamcode.auto.camera.aprilTagsTest;

import org.firstinspires.ftc.teamcode.teleOp.actions.Shooter;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;


@Configurable
@com.qualcomm.robotcore.eventloop.opmode.TeleOp
public class TeleOp extends OpMode {
    @Override
    protected void postInit() {
        Imu.resetYaw();
    }

    @Override
    public void run(){
//        DriveTrain driveTrain = new DriveTrain(DriveBackRight, DriveBackLeft, DriveFrontRight, DriveFrontLeft, telemetry, Imu);
        Shooter shooter = new Shooter(shootMotor,telemetry,shootMotorOp);
        Intake intake = new Intake(IntakeL,IntakeR,telemetry);
        ColorSensorTest cSencor = new ColorSensorTest();
        cSencor.init(hardwareMap);
        boolean is_up = false;
        AprilTagLocalization test = new AprilTagLocalization();
        test.initProcessor(hardwareMap);

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
//            test.telemetryAprilTag(aprilTag);
            test.detectTags();
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
            shooter.shooterTest(gamepad1.dpad_up,gamepad1.dpad_down);


            intake.intakeTest(gamepad1.y);
            if(gamepad1.back){Imu.resetYaw();}

            telemetry.addData("recognized color: ", cSencor.getDetectedColor(telemetry));
            if(test.specialDetection != null){
                telemetry.addData("distance from tag: ", test.distanceToGoal(test.specialDetection.robotPose));
                test.robotToTag=test.distanceToGoal(test.specialDetection.robotPose);
            }
            else{
                telemetry.addData("distance from tag", "null :(((");
            }
            telemetry.addData("Order: ",test.Order);


            telemetry.addData("cam.pose",test.CAM_POS);
            telemetry.addData("outtake power: ",-gamepad1.left_stick_y);
            telemetry.addData("odometry: ",odometry.getCurrentPosition()/tick);
            telemetry.update();
        }

    }

    @Override
    protected void end() {

    }

}
