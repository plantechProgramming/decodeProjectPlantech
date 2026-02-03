package org.firstinspires.ftc.teamcode.teleOp;

import com.acmerobotics.dashboard.config.Config;
import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.OpMode;
import org.firstinspires.ftc.teamcode.teleOp.actions.DriveTrain;
import org.firstinspires.ftc.teamcode.teleOp.actions.Intake;
import org.firstinspires.ftc.teamcode.teleOp.actions.Shooter;
import org.firstinspires.ftc.teamcode.teleOp.actions.Turret;

@Configurable
@Config
@com.qualcomm.robotcore.eventloop.opmode.TeleOp
public class shooterTest extends OpMode {
    @Override
    protected void postInit() {
        odometry.resetPosAndIMU();
    }


    @Override
    protected void run() {
        Shooter shooter = new Shooter(shootMotor,dashboardTelemetry,shootMotorOp);
        Intake intake  = new Intake(intakeIBL,intakeIBR,shooterIBL,shooterIBR,intakeMotor,telemetry);
        Turret turret = new Turret(turretMotor);
        Utils utils = new Utils(telemetry,odometry);
        DriveTrain driveTrain = new DriveTrain(DriveBackRight, DriveBackLeft, DriveFrontRight, DriveFrontLeft, telemetry, Imu,odometry);

        while(opModeIsActive()){
//            turret.turnToDeg(60);
            turret.turret(utils.getAngleFromGoal("BLUE"));
//            turretMotor.setPower(0.2);
            odometry.update();
            shooter.variableSpeedShoot(gamepad1.dpad_up, gamepad1.dpad_down, .02);
            if(gamepad1.a){
                    intake.inBetweenInFull();
                    intake.intakeIn();
            }
//            if(gamepad1.a) shooter.noPhysShoot(0.5);
//            shooter.shooter2.setPower(0.1);
//            dashboardTelemetry.addData("power", shooter.shooter.getPower());
//            telemetry.addData("pow", shooter.shooter.getPower());
            telemetry.addData("pow", turretMotor.getPower());
            telemetry.addData("cur angle", turret.getCurDeg());
            telemetry.addData("cur pos", turretMotor.getCurrentPosition());
            telemetry.addData("cur rev", turret.getRev());
            telemetry.addData("max turret rpm", turretMotor.getMotorType().getMaxRPM());
            telemetry.addData("ticksPerRev", turretMotor.getMotorType().getTicksPerRev());
            dashboardTelemetry.addData("cur position", turretMotor.getCurrentPosition());
            dashboardTelemetry.addData("turretmotor power", turretMotor.getPower());
            shooter.setShooterTelemetry(dashboardTelemetry);
            dashboardTelemetry.update();
            telemetry.update();
        }
    }

    @Override
    protected void end() {

    }
}
