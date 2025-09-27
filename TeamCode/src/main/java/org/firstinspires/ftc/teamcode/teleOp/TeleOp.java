package org.firstinspires.ftc.teamcode.teleOp;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.auto.camera.colorsensor.ColorSensorTest;
import org.firstinspires.ftc.teamcode.teleOp.actions.Elevator;
import org.firstinspires.ftc.teamcode.OpMode;
import org.firstinspires.ftc.teamcode.auto.camera.aprilTagsTest;

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
        Elevator lift = new Elevator(intake_center_angle,IntakeL,IntakeR, telemetry,shooter);
        ColorSensorTest cSencor = new ColorSensorTest();
        cSencor.init(hardwareMap);
        boolean is_up = false;
        aprilTagsTest test = new aprilTagsTest();
        AprilTagProcessor aprilTag = new AprilTagProcessor.Builder()
                .build();

        VisionPortal visionPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam"))
                .addProcessor(aprilTag)
                .build();


        double forward; //-1 to 1
        double turn;
        double drift;
        double botHeading;
        boolean slow = false;
//        EH.setDirection(DcMotorSimple.Direction.REVERSE);
//        EH.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        EA.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


        while (opModeIsActive() ) {
            test.telemetryAprilTag(aprilTag);
            forward = -gamepad1.left_stick_y;
            turn = gamepad1.right_stick_x;
            drift = gamepad1.left_stick_x;
            botHeading = Imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

            ElapsedTime elapsedTime = new ElapsedTime();
//            driveTrain.drive(forward, drift, turn, botHeading, 1);
//            telemetry.addData("x", DriveFrontRight.getCurrentPosition());
//            telemetry.addData("y",DriveBackLeft.getCurrentPosition());
//            tags.initAprilTag();



            if(gamepad1.x && !slow){
//                driveTrain.drive(forward, drift, turn, botHeading, 0.5);
                slow = true;
            }else if(gamepad1.x && slow) {
//                driveTrain.drive(forward, drift, turn, botHeading, 1);
                slow = false;
            }
//            } telemetry.addData("y: ", DriveBackLeft.getCurrentPosition());
//            telemetry.addData("x:", DriveFrontRight.getCurrentPosition());
            lift.intakefunc(-gamepad1.left_stick_y);
//            lift.Shooter(gamepad1.right_bumper);
            telemetry.addData("power",lift.powerU);
            telemetry.addData("powerD",lift.powerD);
//            if (test.Order == "GPP"){
//                lift.intakefunc(true);
//            }else{lift.intakefunc(false);}



            if(gamepad1.back){Imu.resetYaw();}

//            telemetry.addData("ea",EA.getCurrentPosition());
            telemetry.addData("recognized color: ", cSencor.getDetectedColor(telemetry));
            if(test.specialDetection != null){
                telemetry.addData("distance from tag: ", test.robotToTag);
            }
            else{
                telemetry.addData("distance from tag", "null :(((");
            }
            telemetry.addData("Order: ",test.Order);
            telemetry.update();
        }
//        EH.setPower(0);


    }

    @Override
    protected void end() {

    }

}
