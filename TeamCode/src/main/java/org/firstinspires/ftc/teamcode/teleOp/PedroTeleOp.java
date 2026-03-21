package org.firstinspires.ftc.teamcode.teleOp;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.HeadingInterpolator;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.OpMode;
import org.firstinspires.ftc.teamcode.auto.autos.ReadWrite;
import org.firstinspires.ftc.teamcode.auto.pedro.constants.Constants;
import org.firstinspires.ftc.teamcode.teleOp.actions.DriveTrain;
import org.firstinspires.ftc.teamcode.teleOp.actions.Intake;
import org.firstinspires.ftc.teamcode.teleOp.actions.Shooter;
import org.firstinspires.ftc.teamcode.teleOp.actions.Turret;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

import java.util.function.Supplier;
@Configurable
@com.qualcomm.robotcore.eventloop.opmode.TeleOp
public class PedroTeleOp extends OpMode {

    private Follower follower;
    public static Pose startingPose = new Pose(72,72,180);
    private boolean automatedDrive;
    private Supplier<PathChain> pathChain;
    private TelemetryManager telemetryM;
    private boolean slowMode = false;
    private double slowModeMultiplier = 0.5;
    private Pose lastPos;
    private boolean activatedHold = false;


    @Override
    protected void postInit() {

    }

    @Override
    protected void run() {
        odometry.resetPosAndIMU();
        Intake intake  = new Intake(inBetweenMotor,shooterIBL,shooterIBR,intakeMotor,telemetry);
        DriveTrain driveTrain = new DriveTrain(DriveBackRight, DriveBackLeft, DriveFrontRight, DriveFrontLeft, telemetry, Imu,odometry, "RED");
        Shooter shooter = new Shooter(shootMotor,dashboardTelemetry,shootMotorOp, odometry);
        ReadWrite readWrite = new ReadWrite();
        Utils utils = new Utils(telemetry,odometry);
        follower = Constants.createFollower(hardwareMap);
//        follower.setStartingPose(startingPose == null ? new Pose() : startingPose);
        follower.update();
        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();
//        pathChain = () -> follower.pathBuilder() //Lazy Curve Generation
//                .addPath(new Path(new BezierLine(follower::getPose, new Pose(45, 98))))
//                .setHeadingInterpolation(HeadingInterpolator.linearFromPoint(follower::getHeading, Math.toRadians(45), 0.8))
//                .build();
        follower.startTeleopDrive(false);
        lastPos = follower.getPose();
        while(opModeIsActive()){
            if (!gamepad1.right_bumper && !gamepad1.left_bumper) {
                follower.setTeleOpDrive(
                        gamepad1.left_stick_y,
                        gamepad1.left_stick_x,
                        gamepad1.right_stick_x,
                        true);
            }
            //Automated PathFollowing
//            if (gamepad1.aWasPressed()) {
//                follower.followPath(pathChain.get());
//                automatedDrive = true;
//            }
            //Stop automated following if the follower is done
////            if (automatedDrive && (gamepad1.bWasPressed() || !follower.isBusy())) {
////                follower.startTeleopDrive();
////                automatedDrive = false;
////            }
//            if(!gamepad1.right_bumper){
//                lastPos = follower.getPose();
//                if(activatedHold){
//                    activatedHold = false;
//                    follower.startTeleOpDrive(true);
//                }
//            }
//
//            if (gamepad1.right_trigger > 0){
//                intake.intakeIn();
//                intake.inBetweenInPart();
//            }
//            else if(gamepad1.left_trigger!=0) {
//                intake.inBetweenOut();
//                intake.intakeOut();
//            }
//            else if(gamepad1.x){
//                intake.inBetweenOut();
//                intake.intakeOut();
//                shooter.out();
//            }
//            else if(gamepad1.right_bumper){
//                if(shooter.isUpToGivenSpeed(shooter.interpolateTel(utils.getDistFromGoal("BLUE")))){// TODO: change for RED
//                    intake.inBetweenInFull();
//                }
//                intake.intakeIn();
//
//                follower.holdPoint(lastPos);
//                activatedHold = true;
//            }
//            else{
//                intake.stopIntake();
//            }
//
//            if(gamepad1.left_bumper){
//                driveTrain.turnToGoal("BLUE");// TODO: change for RED
//            }
//            if(gamepad1.back){
//                odometry.setPosition(new Pose2D(DistanceUnit.CM,0,0,AngleUnit.DEGREES, 0)); //TODO: change for RED
//            }

            driveTrain.setDriveTelemetry(telemetry);
            driveTrain.setDriveTelemetry(dashboardTelemetry);

            shooter.setShooterTelemetry(telemetry);
            shooter.setShooterTelemetry(dashboardTelemetry);


            telemetryM.debug("position", follower.getPose());
            telemetryM.debug("velocity", follower.getVelocity());
            telemetryM.debug("automatedDrive", automatedDrive);
            telemetry.update();
            dashboardTelemetry.update();
            telemetryM.update();
            odometry.update();
            follower.update();
        }
    }

    @Override
    protected void end() {

    }
}
