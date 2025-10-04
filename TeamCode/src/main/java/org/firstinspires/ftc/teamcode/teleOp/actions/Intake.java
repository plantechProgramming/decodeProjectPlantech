package org.firstinspires.ftc.teamcode.teleOp.actions;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@Configurable
public class Intake {

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

    CRServo IntakeL, IntakeR;
    Telemetry telemetry;
    public double radToTicks = Math.PI/3000;

    // wtf is a type parameter
    public <roni2_intake> Intake(CRServo IntakeL,CRServo IntakeR, Telemetry telemetry){
        this.IntakeL = IntakeL;
        this.IntakeR = IntakeR;
        this.telemetry = telemetry;

    }

    public void intakeTest(boolean pressed){
        if(pressed){
            IntakeR.setPower(1);
            IntakeL.setPower(-1);
        }
        else{
            IntakeL.setPower(0);
            IntakeR.setPower(0);
        }
    }


}

