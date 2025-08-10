package org.firstinspires.ftc.teamcode.teleOp.actions;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.teleOp.PID;
import org.openftc.easyopencv.OpenCvCamera;

public class DriveTrain {
    private DcMotorEx BR, BL, FR, FL;
    private BNO055IMU imu;
    private IMU Imu;
    private Telemetry telemetry;
    private LinearOpMode opMode;
    private OpenCvCamera camera;
    ElapsedTime runtime = new ElapsedTime();
    public static double Kp = 0.5, Ki = 0.2, Kd = 0.01;
    static final double WHEEL_DIAMETER_CM = 9.6;     // For figuring circumference
    static final double COUNTS_PER_CM = 537.6 / WHEEL_DIAMETER_CM * Math.PI;//(COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_CM * PI);

    public DriveTrain(DcMotorEx BR, DcMotorEx BL, DcMotorEx FR, DcMotorEx FL, Telemetry telemetry, IMU imu) {
        this.BL = BL;
        this.BR = BR;
        this.FL = FL;
        this.FR = FR;

        this.Imu = imu;
        this.telemetry = telemetry;
    }


    public void side_drive(double power){
        FL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        FR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        BR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        BL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


        FL.setPower(1 * power);
        BL.setPower(-1 * power);

        FR.setPower(-1 * power);
        BR.setPower(1 * power);
    }

    public void drive(double y, double x, double rx, double botHeading, double slowRatio){

        // slowRatio [0,1] - output power multiplier

        FL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        FR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        BR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        BL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Rotate the movement direction counter to the bot's rotation
        double rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
        double rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);

        rotX *= 1.1;  // Counteract imperfect strafing

        // Denominator is the largest motor power (absolute value) or 1
        // This ensures all the powers maintain the same ratio,
        // but only if at least one is out of the range [-1, 1]
        double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);
        double frontLeftPower = (rotY + rotX + rx) / denominator;
        double backLeftPower = (rotY - rotX + rx) / denominator;

        double frontRightPower = (rotY - rotX - rx) / denominator;// before - rotX
        double backRightPower = (rotY + rotX - rx) / denominator;// before + rotX

        FL.setPower(frontLeftPower * slowRatio);
        BL.setPower(backLeftPower * slowRatio);

        FR.setPower(frontRightPower * slowRatio);
        BR.setPower(backRightPower * slowRatio);

    }



    public void turnToGyro_minus(double degrees) {
        double botAngle = Math.abs(Imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES));
        PID pid = new PID(2, 3, 0, 0, 0);

        pid.setWanted(degrees);

        if(degrees < 0){
            while (botAngle >= degrees) {
                botAngle = Imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);

                FL.setPower(-Math.abs(pid.update(botAngle)));
                FR.setPower(Math.abs(pid.update(botAngle)));

                BR.setPower(Math.abs(pid.update(botAngle)));
                BL.setPower(-Math.abs(pid.update(botAngle)));

                telemetry.addData("IMU", botAngle);
                telemetry.update();

            }stop();
        }
    }

    public void turnToGyro_plus(double degrees) {
        double botAngle = Math.abs(Imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES));
        PID pid = new PID(5, 0, 0, 0, 0);

        BR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        degrees = degrees * (1500.0 / 40.0);

        double pos = BR.getCurrentPosition();
        pid.setWanted(degrees);

        while (Math.abs(pos) <= Math.abs(degrees)) {
                pos = BR.getCurrentPosition();

                FL.setPower(Math.abs(pid.update(pos)));
                BL.setPower(Math.abs(pid.update(pos)));

                FR.setPower(-Math.abs(pid.update(pos)));
                FR.setPower(-Math.abs(pid.update(pos)));

                telemetry.addData("IMU", botAngle);
                telemetry.update();

            }stop();
    }


        public void stop(){
            FL.setPower(0);
            FR.setPower(0);
            BR.setPower(0);
            BL.setPower(0);
        }
        public void GPT_Drive(double x, double y,double botHeading){

            FL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            FR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            BR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            BL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

            double rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
            double rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);

            rotX *= 1.1;
            double wheelRadius = 48.0; // mm
            double maxRPM = 312; // RPM of the GoBilda Yellow Jacket Motors
            double maxLinearSpeed = 1575.98; // mm/s

// Robot Movement Parameters
            double distance = Math.sqrt((Math.pow(rotX,2))+Math.pow(rotY,2)); // mm (example distance)

// Convert angle to radians
            double theta = Math.abs(Math.asin(rotY/distance)); // radians (example angle)

// Decompose the distance into x and y components
//            double v_x = distance * Math.cos(theta);  // Forward/Backward component
//            double v_y = distance * Math.sin(theta);  // Strafe component

// Calculate wheel speeds (in mm/s)
            double v_FL = rotY - rotX;
            double v_FR = rotY - rotX;
            double v_BL = rotY + rotX;
            double v_BR = rotY + rotX;

// Convert wheel speeds to motor RPMs
//            double rpm_FL = (Math.abs(v_FL) / maxLinearSpeed) * maxRPM;
//            double rpm_FR = (Math.abs(v_FR) / maxLinearSpeed) * maxRPM;
//            double rpm_BL = (Math.abs(v_BL) / maxLinearSpeed) * maxRPM;
//            double rpm_BR = (Math.abs(v_BR) / maxLinearSpeed) * maxRPM;

// Normalize the motor power to be between -1 and 1
            double motorPower_FL = v_FL / maxLinearSpeed;
            double motorPower_FR = v_FR / maxLinearSpeed;
            double motorPower_RL = v_BL / maxLinearSpeed;
            double motorPower_RR = v_BR / maxLinearSpeed;

// Cap motor power to the range [-1, 1]
            motorPower_FL = Range.clip(motorPower_FL, -1.0, 1.0);
            motorPower_FR = Range.clip(motorPower_FR, -1.0, 1.0);
            motorPower_RL = Range.clip(motorPower_RL, -1.0, 1.0);
            motorPower_RR = Range.clip(motorPower_RR, -1.0, 1.0);

// Output motor powers to the robot motors
            FL.setPower(motorPower_FL);
            FR.setPower(motorPower_FR);
            BL.setPower(motorPower_RL);
            BR.setPower(motorPower_RR);
            telemetry.addData("FL_Power:", motorPower_FL);
            telemetry.addData("BL_Power:", motorPower_RL);
            telemetry.addData("BR_Power:", motorPower_RR);
            telemetry.addData("FR_Power:", motorPower_FR);
            telemetry.update();


        }
}
