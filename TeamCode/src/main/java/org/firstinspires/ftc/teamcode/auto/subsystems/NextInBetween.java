package org.firstinspires.ftc.teamcode.auto.subsystems;

import static com.pedropathing.ivy.groups.Groups.parallel;

import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.commands.Commands;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import dev.nextftc.core.commands.groups.ParallelGroup;
import dev.nextftc.hardware.powerable.SetPower;

public class NextInBetween{
    CRServo sl, sr;
    DcMotorEx inBetweenMotor;
    double IN_POWER = 1;
    double OUT_POWER = -0.5;
    double STOP_POWER = 0;
    double STOP_POWER_MOTOR = 0;
    double IN_POWER_MOTOR = 0.95;

    public NextInBetween(HardwareMap hardwareMap){
        initHardware(hardwareMap);
    }

    private void initHardware(HardwareMap hardwareMap){
        inBetweenMotor = hardwareMap.get(DcMotorEx.class, "inbetween");
        inBetweenMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        inBetweenMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        inBetweenMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        sl = hardwareMap.get(CRServo.class,"SIBR");
        sr = hardwareMap.get(CRServo.class,"SIBL");
    }

    public Command inBetweenInFull(){
        return parallel(
                inShooterPrimers(),
                setMotorPowerAsCommand(IN_POWER_MOTOR)
        );
    }

    public Command inBetweenInPart(){
        return parallel(
                setPrimerPowerAsCommand(OUT_POWER),
                setMotorPowerAsCommand(IN_POWER_MOTOR)
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
