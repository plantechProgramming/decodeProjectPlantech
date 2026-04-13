package org.firstinspires.ftc.teamcode.auto.subsystems;

import com.pedropathing.ivy.CommandBuilder;
import com.pedropathing.ivy.commands.Commands;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class NextIntake{
    DcMotorEx intakeMotor;
    double STOP_POWER = 0;
    double OUT_POWER = -1;
    double IN_POWER = 1;
    public NextIntake(HardwareMap hardwareMap) {

    }

    public CommandBuilder take(){
        return Commands.instant(()->intakeMotor.setPower(IN_POWER));
    }
    public CommandBuilder out(){
        return Commands.instant(()->intakeMotor.setPower(OUT_POWER));
    }
    public CommandBuilder stop(){
        return Commands.instant(()->intakeMotor.setPower(STOP_POWER));
    }

    public CommandBuilder setPowerAsCommand(double pow){
        return Commands.instant(()->intakeMotor.setPower(pow));
    }
}
