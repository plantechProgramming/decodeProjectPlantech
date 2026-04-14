package org.firstinspires.ftc.teamcode.auto.subsystems;


import static com.pedropathing.ivy.groups.Groups.parallel;

import static org.firstinspires.ftc.teamcode.teleOp.actions.Shooter.kD;
import static org.firstinspires.ftc.teamcode.teleOp.actions.Shooter.kF;
import static org.firstinspires.ftc.teamcode.teleOp.actions.Shooter.kI;
import static org.firstinspires.ftc.teamcode.teleOp.actions.Shooter.kP;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.CommandBuilder;
import com.pedropathing.ivy.commands.Commands;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.teleOp.PID;
import org.firstinspires.ftc.teamcode.teleOp.actions.GetVelocity;

import dev.nextftc.control.ControlSystem;
import dev.nextftc.control.KineticState;
import dev.nextftc.control.feedforward.FeedforwardElement;
import dev.nextftc.core.commands.groups.ParallelDeadlineGroup;
import dev.nextftc.core.commands.groups.ParallelGroup;
import dev.nextftc.core.commands.groups.SequentialGroup;
import dev.nextftc.core.commands.utility.InstantCommand;
import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.core.units.Distance;
import dev.nextftc.ftc.NextFTCOpMode;
import dev.nextftc.hardware.delegates.Caching;
import dev.nextftc.hardware.impl.MotorEx;
import dev.nextftc.hardware.impl.ServoEx;
import dev.nextftc.hardware.positionable.SetPosition;
import dev.nextftc.hardware.powerable.SetPower;
@Configurable
@Config
public class NextShooter{
    DcMotorEx shootMotor, shootMotorOp;
    GetVelocity shooterVel;

    public NextShooter(HardwareMap hardwareMap) {
        initHardware(hardwareMap);
        shooterVel = new GetVelocity(shootMotor, 0.1);
    }

    private void initHardware(HardwareMap hardwareMap){
        shootMotor = hardwareMap.get(DcMotorEx.class, "shooter");
        shootMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        shootMotor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);

        shootMotorOp = hardwareMap.get(DcMotorEx.class, "shooter2");
        shootMotorOp.setDirection(DcMotorSimple.Direction.FORWARD);
        shootMotorOp.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
    }

    double Szonedis = 0.5;
    public static double farPow = 0.541;
    public static double closePow = 0.387;
    PID controller = new PID(kP,kI,kD,kF);
    public Command naiveShooter(boolean far) {
        if (!far) {
            Szonedis = closePow;
        } else {
            Szonedis = farPow;
        }
        controller.setWanted(Szonedis);
        double output = controller.update(shooterVel.getVelocityFilter()/6000);
        return parallel(
                Commands.instant(()->shootMotor.setPower(output)),
                Commands.instant(()->shootMotorOp.setPower(-output))
        );
    }

    public void updateTelemetry(Telemetry telemetry){
        telemetry.addData("current vel",shooterVel.getVelocityFilter());
        telemetry.addData("wanted vel", Szonedis);
        telemetry.update();
    }
}
