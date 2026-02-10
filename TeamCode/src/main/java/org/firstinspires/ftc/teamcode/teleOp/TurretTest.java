package org.firstinspires.ftc.teamcode.teleOp;

import com.acmerobotics.dashboard.config.Config;
import com.bylazar.configurables.annotations.Configurable;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.OpMode;
import org.firstinspires.ftc.teamcode.auto.autos.ReadWrite;
import org.firstinspires.ftc.teamcode.teleOp.actions.DriveTrain;
import org.firstinspires.ftc.teamcode.teleOp.actions.Intake;
import org.firstinspires.ftc.teamcode.teleOp.actions.Shooter;
import org.firstinspires.ftc.teamcode.teleOp.actions.Turret;

@Configurable
@Config
@com.qualcomm.robotcore.eventloop.opmode.TeleOp
public class TurretTest extends OpMode {
    @Override
    protected void postInit() {
        odometry.resetPosAndIMU();
    }
    @Override
    protected void run() {
        Shooter shooter = new Shooter(shootMotor,dashboardTelemetry,shootMotorOp);
        Intake intake  = new Intake(intakeIBL,intakeIBR,shooterIBL,shooterIBR,intakeMotor,telemetry);
        Turret turret = new Turret(turretMotor, odometry);
        Utils utils = new Utils(telemetry,odometry);
        ReadWrite readWrite = new ReadWrite();
        DriveTrain driveTrain = new DriveTrain(DriveBackRight, DriveBackLeft, DriveFrontRight, DriveFrontLeft, telemetry, Imu,odometry);

        sleep(100);
        double forward; //-1 to 1
        double turn;
        double drift;
        double botHeading;
        boolean slow = false;
        double tick = 2000/(48*Math.PI); //per tick
        odometry.setPosition(utils.PedroPoseConverter(readWrite.readPose()));
        boolean turretActivated = false;

        while(opModeIsActive()){
            forward = -gamepad1.left_stick_y;
            turn = gamepad1.right_stick_x;
            drift = gamepad1.left_stick_x;
            //todo: pinpoint
            botHeading = odometry.getHeading(AngleUnit.RADIANS);

//            turret.turnToDeg(60);
            driveTrain.drive(forward, drift, turn, botHeading, 1);//TODO: change for RED
            if(gamepad1.y && !turretActivated){
                turretActivated = true;
            }
            if(turretActivated){
                turret.turnToDegCorrected(utils.getAngleFromGoal("BLUE"));
            }
//            turretMotor.setPower(0.2);
            odometry.update();
//            if(gamepad1.a){
//                if(shooter.isUpToSpeed()){
//                    shooter.naiveShooter(driveTrain.isFar());
//                }
//            }
//            if(gamepad1.a) shooter.noPhysShoot(0.5);
//            shooter.shooter2.setPower(0.1);
//            dashboardTelemetry.addData("power", shooter.shooter.getPower());
//            telemetry.addData("pow", shooter.shooter.getPower());
            turret.setTelemetry(telemetry);
            turret.setTelemetry(dashboardTelemetry);
            dashboardTelemetry.update();
            telemetry.update();
        }
    }

    @Override
    protected void end() {

    }
}
