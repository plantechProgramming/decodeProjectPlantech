package org.firstinspires.ftc.teamcode.teleOp.actions;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.teleOp.PID;

@Config
public class Elevator{

    public static double kP_EH = 0.15;
    public static double kI_EH = 0.05;
    public static double kD_EH = 0.05;

    public static double kP_intA = 0.15;
    public static double kI_intA = 0.05;
    public static double kD_intA = 0.05;

    public static double kP_EA = 0.03;
    public static double kI_EA = 0.01;
    public static double kD_EA = 0.01;

    PID pid_EH = new PID(kP_EH, kI_EH, kD_EH, 0, 0);
    PID pid_intA = new PID(kP_intA, kI_intA, kD_intA, 0, 0);
    PID pid_EA = new PID(kP_EA, kI_EA, kD_EA, 0,0);

    public static double thresh = 80;
    public double wanted;
    public double intakeWanted;
    //Thread thread = Thread.currentThread();
    ElapsedTime runtime = new ElapsedTime();
    DcMotorEx EH, EA;
    Servo intake_center_angle;
    CRServo IntakeL, IntakeR;
    Telemetry telemetry;
    public double radToTicks = Math.PI/3000;

    // wtf is a type parameter
    public <roni2_intake> Elevator(DcMotorEx EA, DcMotorEx EH, Servo intake_center_angle,CRServo IntakeL,CRServo IntakeR, Telemetry telemetry){
        this.EH = EH;
        this.EA = EA;
        this.IntakeL = IntakeL;
        this.IntakeR = IntakeR;
        this.intake_center_angle = intake_center_angle;
        this.telemetry = telemetry;
        EA.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        EH.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        EA.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        EH.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
    }


    public void Intake(boolean in,boolean out){
         if (in){
            IntakeL.setPower(1);
            IntakeR.setPower(-1);
//            pid_EA.setWanted(EA.getCurrentPosition());
        } else if (out){
            IntakeL.setPower(-1);
            IntakeR.setPower(1);
//            pid_EA.setWanted(EA.getCurrentPosition());
        } else if (!in && !out) {
             IntakeR.setPower(0);
             IntakeL.setPower(0);
//        } else {
//            double power_EA = pid_EA.update(EA.getCurrentPosition());
             EH.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
             EH.setPower(0);
         }
    }
    public void intakefunc(boolean dir){
       if(dir == true) {
           IntakeL.setPower(1);
           IntakeR.setPower(-1);
       }
    }
    public void turnOffIntake(){
           IntakeL.setPower(0);
           IntakeR.setPower(0);
    }
}

