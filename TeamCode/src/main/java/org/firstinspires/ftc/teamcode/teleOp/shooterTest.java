package org.firstinspires.ftc.teamcode.teleOp;

import com.acmerobotics.dashboard.config.Config;
import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.OpMode;
import org.firstinspires.ftc.teamcode.auto.autos.ReadWrite;
import org.firstinspires.ftc.teamcode.teleOp.actions.DriveTrain;
import org.firstinspires.ftc.teamcode.teleOp.actions.GetVelocity;
import org.firstinspires.ftc.teamcode.teleOp.actions.Intake;
import org.firstinspires.ftc.teamcode.teleOp.actions.Shooter;
import org.firstinspires.ftc.teamcode.teleOp.actions.Turret;

import java.util.Arrays;

@Configurable
@Config
@com.qualcomm.robotcore.eventloop.opmode.TeleOp(group = "tests")
public class shooterTest extends OpMode {
    @Override
    protected void postInit() {
        odometry.resetPosAndIMU();
    }


    @Override
    protected void run() {
        Shooter shooter = new Shooter(shootMotor,dashboardTelemetry,shootMotorOp, odometry);
        Intake intake  = new Intake(inBetweenMotor,shooterIBL,shooterIBR,intakeMotor,telemetry);
        ElapsedTime elapsedTime = new ElapsedTime();
        GetVelocity shooterVelocity = new GetVelocity(shootMotor,0.1, 8192);
//        Turret turret = new Turret(turretMotor, odometry);
        ReadWrite readWrite = new ReadWrite();
        Utils utils = new Utils(telemetry,odometry);
        odometry.setPosition(new Pose2D(DistanceUnit.CM,0,0,AngleUnit.DEGREES, 180)); //TODO: change for RED
        DriveBackLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        DriveBackRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        DriveFrontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        DriveFrontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        while(opModeIsActive()){
            elapsedTime.reset();
//            double forward = -gamepad1.left_stick_y;
//            double turn = gamepad1.right_stick_x;
//            double drift = gamepad1.left_stick_x;
            shooter.variableSpeedShoot(gamepad1.dpad_up, gamepad1.dpad_down, 0.01, voltageSensor.getVoltage());
//
////            shooter.shooter.setPower(0.3);
////            shooter.shooter2.setPower(-0.3);
////            shooter.noPhysShootHomeostasis(0.5);
//
            if(gamepad1.a){
               if(shooter.isUpToGivenSpeed(shooter.power, "RED")){
                   intake.inBetweenInFull();
                   intake.intakeIn();
               }
           }
//
//
////           if(gamepad1.b){
////               shooter.noPhysShoot(0.5);
////           }
////            if(gamepad1.a) shooter.noPhysShoot(0.5);
////            shooter.shooter2.setPower(0.1);
//
            shooter.setShooterTelemetry(dashboardTelemetry);
            shooter.setShooterTelemetry(telemetry);
            telemetry.update();
            dashboardTelemetry.update();
            odometry.update();
            updateHisto(elapsedTime.milliseconds());
        }
    }

    @Override
    protected void end() {
        System.out.println(Arrays.toString(getHisto()));
        sleep(10000);
    }

    int[] arr = new int[5];
    public void updateHisto(double loopTime){
        if(0 <= loopTime && loopTime <= 0.5){
            arr[0]++;
        }
        else if(0.5 < loopTime && loopTime <= 1){
            arr[1]++;
        }
        else if(1 < loopTime && loopTime <= 10){
            arr[2]++;
        }
        else if(10 < loopTime && loopTime <= 20){
            arr[3]++;
        }
        else{
            arr[4]++;
        }

    }

    public int[] getHisto(){
        return arr;
    }
}
