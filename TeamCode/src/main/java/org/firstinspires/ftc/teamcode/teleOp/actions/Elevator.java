package org.firstinspires.ftc.teamcode.teleOp.actions;

import com.acmerobotics.dashboard.config.Config;
import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.teleOp.PID;

@Configurable
public class Elevator{

//    public static double kP_EH = 0.15;
//    public static double kI_EH = 0.05;
//    public static double kD_EH = 0.05;
//
//    public static double kP_intA = 0.15;
//    public static double kI_intA = 0.05;
//    public static double kD_intA = 0.05;
//
//    public static double kP_EA = 0.03;
//    public static double kI_EA = 0.01;
//    public static double kD_EA = 0.01;
    public static double powerU = 0.5;
    public static double powerD = 0.5;

//    PID pid_EH = new PID(kP_EH, kI_EH, kD_EH, 0, 0);
//    PID pid_intA = new PID(kP_intA, kI_intA, kD_intA, 0, 0);
//    PID pid_EA = new PID(kP_EA, kI_EA, kD_EA, 0,0);

    public static double thresh = 80;
    public double wanted;
    public double intakeWanted;
    //Thread thread = Thread.currentThread();
    ElapsedTime runtime = new ElapsedTime();
    DcMotorEx EH, EA,SU,SD, shooter;
    Servo intake_center_angle;
    CRServo IntakeL, IntakeR;
    Telemetry telemetry;
    public double radToTicks = Math.PI/3000;

    // wtf is a type parameter
    public <roni2_intake> Elevator(Servo intake_center_angle,CRServo IntakeL,CRServo IntakeR, Telemetry telemetry, DcMotorEx shooter){
        this.shooter = shooter;
        this.IntakeL = IntakeL;
        this.IntakeR = IntakeR;
        this.intake_center_angle = intake_center_angle;
        this.telemetry = telemetry;

    }

    public void intake(boolean pressed){
        if(pressed){
            IntakeR.setPower(1);
            IntakeL.setPower(-1);
        }
        else{
            IntakeL.setPower(0);
            IntakeR.setPower(0);
        }
    }

    public void intakefunc(double power){
       shooter.setPower(power);
    }
}

