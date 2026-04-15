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

@Configurable
@Config
public class NextShooter{
    DcMotorEx shootMotor, shootMotorOp;
    GetVelocity shooterVel;
    double MAX_RPM = 6000;
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

    double wantedPow;
    public static double farPow = 0.541;
    public static double closePow = 0.387;
    PID controller = new PID(kP,kI,kD,kF);

    public Command naiveShooter(boolean far) {
        if (far) {
            wantedPow = farPow;
        } else {
            wantedPow = closePow;
        }
        controller.setWanted(wantedPow);
        double output = controller.update(getCurPower());
        return setShooterPowerAsCommand(output);
    }

    public void updateTelemetry(Telemetry telemetry){
        telemetry.addData("current pow",getCurPower());
        telemetry.addData("wanted pow", wantedPow);
        telemetry.update();
    }

    public Command setShooterPowerAsCommand(double pow){
        return parallel(
                Commands.instant(()->shootMotor.setPower(pow)),
                Commands.instant(()->shootMotorOp.setPower(-pow))
        );
    }

    public double getCurPower(){
        return shooterVel.getVelocityFilter()/MAX_RPM;
    }

    public void periodic(){
        controller.update(getCurPower());
    }
}
