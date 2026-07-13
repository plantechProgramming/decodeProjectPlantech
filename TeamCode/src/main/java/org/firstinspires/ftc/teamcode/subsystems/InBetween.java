package org.firstinspires.ftc.teamcode.subsystems;

import static com.pedropathing.ivy.groups.Groups.parallel;

import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.commands.Commands;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.Misc.InitMotors;

public class InBetween {
    CRServo sl, sr;
    DcMotorEx inBetweenMotor;
    double IN_POWER = 1;
    double OUT_POWER = -0.5;
    double STOP_POWER = 0;
    double STOP_POWER_MOTOR = 0;
    double IN_POWER_MOTOR = 0.95;


    public InBetween(){
        inBetweenMotor = InitMotors.inBetweenMotor;
        this.sl = InitMotors.SL;
        this.sr = InitMotors.SR;
    }

    public Command inFull(){
        return parallel(
                inShooterPrimers(),
                setMotorPowerAsCommand(IN_POWER_MOTOR)
        );
    }

    public Command inPart(){
        return parallel(
                setPrimerPowerAsCommand(OUT_POWER),
                setMotorPowerAsCommand(IN_POWER_MOTOR)
        );
    }

    public Command out(){
        return parallel(
                setPrimerPowerAsCommand(OUT_POWER),
                setMotorPowerAsCommand(OUT_POWER)
        );
    }

    public Command stop(){
        return parallel(
                stopShooterPrimers(),
                setMotorPowerAsCommand(STOP_POWER_MOTOR)
        );
    }

    public Command stopShooterPrimers(){
        return setPrimerPowerAsCommand(STOP_POWER);
    }

    public Command inShooterPrimers(){
        return setPrimerPowerAsCommand(IN_POWER);
    }

    public Command setPrimerPowerAsCommand(double pow){
        return parallel(
                Commands.instant(()->sl.setPower(-pow)),
                Commands.instant(()->sr.setPower(pow))
        );
    }

    public Command setMotorPowerAsCommand(double pow){
        return Commands.instant(()->inBetweenMotor.setPower(pow));
    }
}
