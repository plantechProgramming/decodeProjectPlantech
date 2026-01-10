package org.firstinspires.ftc.teamcode.auto.subsystems;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.auto.camera.AprilTagLocalization;
import org.firstinspires.ftc.teamcode.teleOp.actions.Shooter;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagGameDatabase;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import dev.nextftc.control.ControlSystem;
import dev.nextftc.control.KineticState;
import dev.nextftc.control.feedforward.FeedforwardElement;
import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.groups.ParallelDeadlineGroup;
import dev.nextftc.core.commands.groups.ParallelGroup;
import dev.nextftc.core.commands.groups.SequentialGroup;
import dev.nextftc.core.commands.utility.InstantCommand;
import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.core.units.Distance;
import dev.nextftc.hardware.impl.MotorEx;
import dev.nextftc.hardware.impl.ServoEx;
import dev.nextftc.hardware.positionable.SetPosition;
import dev.nextftc.hardware.powerable.SetPower;

public class NextShooter implements Subsystem {
    public static final NextShooter INSTANCE = new NextShooter();
    public NextShooter() { }
    FtcDashboard dashboard = FtcDashboard.getInstance();
    Telemetry dashboardTelemetry =  dashboard.getTelemetry();
    private DcMotorEx shooter1;
    private DcMotorEx shooter2;
    double Szonedis = 0.55;
    double errorFix = 1.375;

    PIDFCoefficients pidNew = new PIDFCoefficients(30,0,0,0);

    public Command naiveShooter(boolean far) {
        if (!far) {
            Szonedis = .47;
        } else {
            Szonedis = 0.56;
        }
//        controlSystem.setGoal(new KineticState(0,Szonedis*errorFix,0));

        return new InstantCommand(
                        () -> {
                            shooter1.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidNew);
                            shooter2.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidNew);
                            shooter1.setPower(Szonedis*errorFix);
                            shooter2.setPower(-Szonedis*errorFix);
                            dashboardTelemetry.addData("power: ", shooter1.getPower());
                            dashboardTelemetry.addData("Szonedis*errorFix",Szonedis*errorFix);
                            dashboardTelemetry.update();
                        }

                );
    }

    // UNUSED. should check if needed, uses nextControl instead of built in motor pid
    ControlSystem controlSystem = ControlSystem.builder()//next pid
            .posPid(65, 1, 8)
            .build();

    public Command setPowerPID(MotorEx motor, MotorEx motor2){
        KineticState state = new KineticState(motor.getCurrentPosition(), motor.getVelocity(),0);
        KineticState state2 = new KineticState(motor2.getCurrentPosition(), motor2.getVelocity(),0);
        return new ParallelGroup(
                new SetPower(motor, controlSystem.calculate(state)),
                new SetPower(motor2, controlSystem.calculate(state2))
        );
    }
}
