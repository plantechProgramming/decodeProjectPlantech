package org.firstinspires.ftc.teamcode.teleOp;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.teleOp.actions.DriveTrain;
import org.firstinspires.ftc.teamcode.teleOp.actions.Elevator;
import org.firstinspires.ftc.teamcode.OpMode;

import android.util.Size;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.opencv.ColorBlobLocatorProcessor;
import org.firstinspires.ftc.vision.opencv.ColorRange;
import org.firstinspires.ftc.vision.opencv.ImageRegion;
import org.firstinspires.ftc.vision.opencv.PredominantColorProcessor;
import org.opencv.core.RotatedRect;

import java.util.List;


@Config
@com.qualcomm.robotcore.eventloop.opmode.TeleOp
public class TeleOp extends OpMode {
    @Override
    protected void postInit() {
        Imu.resetYaw();
    }

    @Override
    public void run(){
        DriveTrain driveTrain = new DriveTrain(DriveBackRight, DriveBackLeft, DriveFrontRight, DriveFrontLeft, telemetry, Imu);
        Elevator lift = new Elevator(EA, EH, intake_center_angle,IntakeL,IntakeR, telemetry);

        boolean is_up = false;

        double forward; //-1 to 1
        double turn;
        double drift;
        double botHeading;
        boolean slow = false;
        EH.setDirection(DcMotorSimple.Direction.REVERSE);
        EH.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        EA.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


        while (opModeIsActive() ) {
            forward = -gamepad1.left_stick_y;
            turn = gamepad1.right_stick_x;
            drift = gamepad1.left_stick_x;
            botHeading = Imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

            ElapsedTime elapsedTime = new ElapsedTime();
            driveTrain.drive(forward, drift, turn, botHeading, 1);
            telemetry.addData("x", DriveFrontRight.getCurrentPosition());
            telemetry.addData("y",DriveBackLeft.getCurrentPosition());


            if(gamepad1.x && !slow){
                driveTrain.drive(forward, drift, turn, botHeading, 0.5);
                slow = true;
            }else if(gamepad1.x && slow) {
                driveTrain.drive(forward, drift, turn, botHeading, 1);
                slow = false;

            } telemetry.addData("y: ", DriveBackLeft.getCurrentPosition());
            telemetry.addData("x:", DriveFrontRight.getCurrentPosition());


            lift.Intake(gamepad1.a,gamepad1.b);

            if(gamepad1.y){lift.preload();}


//            lift.Intake_Angle(gamepad1.dpad_down,gamepad1.dpad_up);
            lift.Intake_Angle(gamepad1.dpad_down,gamepad1.dpad_up);
//            if (gamepad1.dpad_left){intake_center_angle.setPosition(1);}
//            if(gamepad2.dpad_right){intake_center_angle.setPosition(0);}

//            intake_center_angle.setPosition(0.57);

//            if (gamepad1.right_trigger>0){lift.set_wanted_height(2900); telemetry.addData("wanted", lift.wanted);}
//            else if (gamepad1.left_trigger>0) {lift.set_wanted_height(0);}
            lift.heightByPress(gamepad1.right_trigger,gamepad1.left_trigger);

            lift.Change_Angle(gamepad1.right_bumper,gamepad1.left_bumper);

            if(gamepad1.back){Imu.resetYaw();}

            telemetry.addData("ea",EA.getCurrentPosition());

            telemetry.update();
        }
        EH.setPower(0);


    }

    @Override
    protected void end() {

    }

}
