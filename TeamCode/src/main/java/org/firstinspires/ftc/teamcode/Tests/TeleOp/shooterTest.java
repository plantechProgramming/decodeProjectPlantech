package org.firstinspires.ftc.teamcode.Tests.TeleOp;

import com.acmerobotics.dashboard.config.Config;
import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.TeamOpMode;
import org.firstinspires.ftc.teamcode.Misc.Txt.ReadWrite;
import org.firstinspires.ftc.teamcode.subsystems.DriveTrain;

@Configurable
@Config
@com.qualcomm.robotcore.eventloop.opmode.TeleOp
public class shooterTest extends TeamOpMode {
    @Override
    protected void postInit() {
        odometry.resetPosAndIMU();
    }


    @Override
    protected void run() {
        Shooter shooter = new Shooter(shootMotor,dashboardTelemetry,shootMotorOp, odometry);
        Intake intake  = new Intake(inBetweenMotor,shooterIBL,shooterIBR,intakeMotor,telemetry);
//        Turret turret = new Turret(turretMotor, odometry);
        ReadWrite readWrite = new ReadWrite();
        DriveTrain driveTrain = new DriveTrain(DriveBackRight, DriveBackLeft, DriveFrontRight, DriveFrontLeft, telemetry, Imu,odometry, "RED");
        odometry.setPosition(new Pose2D(DistanceUnit.CM,0,0,AngleUnit.DEGREES, 180)); //TODO: change for RED
        DriveBackLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        DriveBackRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        DriveFrontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        DriveFrontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        while(opModeIsActive()){
            double forward = -gamepad1.left_stick_y;
            double turn = gamepad1.right_stick_x;
            double drift = gamepad1.left_stick_x;
            shooter.variableSpeedShoot(gamepad1.dpad_up, gamepad1.dpad_down, .01);
            driveTrain.drive(-forward, -drift, turn, odometry.getHeading(AngleUnit.DEGREES), 1);//TODO: change for RED -forward, -drift

            if(gamepad1.a){
               if(shooter.isUpToGivenSpeed(shooter.power)){
                   intake.inBetweenInFull();
                   intake.intakeIn();
               }
           }

            shooter.setShooterTelemetry(dashboardTelemetry);
            shooter.setShooterTelemetry(telemetry);
            driveTrain.setDriveTelemetry(telemetry);
            driveTrain.setDriveTelemetry(dashboardTelemetry);
            telemetry.update();
            dashboardTelemetry.update();
            odometry.update();

        }
    }

    @Override
    protected void end() {

    }
}
