package org.firstinspires.ftc.teamcode.teleOp.actions;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@Configurable
public class Intake {

    public static double powerU = 0.5;
    public static double powerD = 0.5;

    public static double thresh = 80;
    public double wanted;
    public double intakeWanted;
    //Thread thread = Thread.currentThread();
    ElapsedTime runtime = new ElapsedTime();

    public CRServo ibl,ibr,sl,sr; //ib = inbetween, s = shooter
    public DcMotorEx intake_motor;
    Telemetry telemetry;
    public double radToTicks = Math.PI/3000;

    // wtf is a type parameter
    public Intake(CRServo ibl,CRServo ibr,CRServo sl,CRServo sr, DcMotorEx intake_motor ,Telemetry telemetry){
        this.telemetry = telemetry;
        this.ibl=ibl;
        this.ibr=ibr;
        this.sl=sl;
        this.sr=sr;
        this.intake_motor = intake_motor;
    }

    public void inBetweenFunc(boolean in,boolean out){
        if(in){
            ibr.setPower(1);
            ibl.setPower(-1);
            sr.setPower(1);
            sl.setPower(-1);
        }
        else if(out){
            ibr.setPower(-1);
            ibl.setPower(1);
            sr.setPower(-1);
            sl.setPower(1);
        }
        else{
            ibl.setPower(0);
            ibr.setPower(0);
            sr.setPower(0);
            sl.setPower(0);
        }
    }
    public void inBetweenInFull(){
        ibr.setPower(-1);
        ibl.setPower(1);
        sr.setPower(-1);
        sl.setPower(1);
    }
    public void inBetweenInPart(){
        ibr.setPower(-1);
        ibl.setPower(1);
        sr.setPower(1);
        sl.setPower(-1);

    }
    public void inBetweenOut(){
        ibr.setPower(1);
        ibl.setPower(-1);
        sr.setPower(1);
        sl.setPower(-1);
    }
    public void intakeFunc(boolean in, boolean out){ //motor is flipped
        if(in){
            intake_motor.setPower(.92);
        }
        else if(out){
            intake_motor.setPower(-.92);
        }
        else{
            intake_motor.setPower(0);
        }
    }
    public void intakeIn(){
        intake_motor.setPower(.92);
    }
    public void intakeOut(){
        intake_motor.setPower(-.92);
    }
    public void stopIntake(){
        intake_motor.setPower(0);
        ibr.setPower(0);
        ibl.setPower(0);
        sr.setPower(0);
        sl.setPower(0);
    }
}

