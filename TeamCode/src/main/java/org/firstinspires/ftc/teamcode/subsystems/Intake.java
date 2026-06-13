package org.firstinspires.ftc.teamcode.subsystems;

import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.commands.Commands;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.Misc.InitMotors;

public class Intake {
    DcMotorEx intakeMotor;
    double STOP_POWER = 0;
    double OUT_POWER = -1;
    double IN_POWER = 1;

    public Intake() {
        intakeMotor = InitMotors.intakeMotor;
    }

    public Command take(){
        return setPowerAsCommand(IN_POWER);
    }
    public Command out(){
        return setPowerAsCommand(OUT_POWER);
    }
    public Command stop(){
        return setPowerAsCommand(STOP_POWER);
    }

    public Command setPowerAsCommand(double pow){
        return Commands.instant(()->intakeMotor.setPower(pow));
    }
}
