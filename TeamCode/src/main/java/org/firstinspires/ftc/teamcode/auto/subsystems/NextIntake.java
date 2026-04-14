package org.firstinspires.ftc.teamcode.auto.subsystems;

import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.CommandBuilder;
import com.pedropathing.ivy.commands.Commands;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class NextIntake{
    DcMotorEx intakeMotor;
    double STOP_POWER = 0;
    double OUT_POWER = -1;
    double IN_POWER = 1;
    public NextIntake(HardwareMap hardwareMap) {
        initHardware(hardwareMap);
    }

    private void initHardware(HardwareMap hardwareMap){
        intakeMotor = hardwareMap.get(DcMotorEx.class,"Intake");
        intakeMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        intakeMotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        intakeMotor.setDirection(DcMotorSimple.Direction.REVERSE);
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
