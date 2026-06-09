package org.firstinspires.ftc.teamcode.subsystems;

import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.commands.Commands;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.InitMotors;

public class Intake {
    DcMotorEx intakeMotor;
    double STOP_POWER = 0;
    double OUT_POWER = -1;
    double IN_POWER = 1;

    public Intake() {
        intakeMotor = InitMotors.intakeMotor;
    }

    public Command take(){
        return Commands.instant(()->intakeMotor.setPower(IN_POWER));
    }
    public Command out(){
        return Commands.instant(()->intakeMotor.setPower(OUT_POWER));
    }
    public Command stop(){
        return Commands.instant(()->intakeMotor.setPower(STOP_POWER));
    }

    public Command setPowerAsCommand(double pow){
        return Commands.instant(()->intakeMotor.setPower(pow));
    }
}
