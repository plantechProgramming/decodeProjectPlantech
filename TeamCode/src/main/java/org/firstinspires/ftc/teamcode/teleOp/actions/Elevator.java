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

    public void Move_Elevator(double x){

        EH.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        PID pid = new PID(0.5, 0.2, 0.1, 0, 0);

        EH.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        pid.setWanted(x);

        while(Math.abs(x) + 320 > Math.abs(EH.getCurrentPosition()) && Math.abs(EH.getCurrentPosition()) < Math.abs(x) - 320){
            EH.setPower(pid.update(EH.getCurrentPosition()));
            telemetry.addLine("in");
            telemetry.addData("pos: ", EH.getCurrentPosition());
            telemetry.addData("x: ", x);
            telemetry.update();
        }

        EH.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        EH.setPower(0.0005);

    }

    public void Move_Elevator_Angle(double x){

        EA.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        PID pid = new PID(0.5, 0.2, 0.1, 0, 0);

        EA.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        pid.setWanted(x);

        while(Math.abs(x) + 320 > Math.abs(EA.getCurrentPosition()) && Math.abs(EA.getCurrentPosition()) < Math.abs(x) - 320){
            EA.setPower(pid.update(EA.getCurrentPosition()));
            telemetry.addLine("in");
            telemetry.addData("pos: ", EA.getCurrentPosition());
            telemetry.addData("x: ", x);
            telemetry.update();
        }

        EA.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        EA.setPower(0.0005);

    }

    public void Change_Angle(boolean right, boolean left){
        double alpha = (EA.getCurrentPosition()*radToTicks);
        //equation: m*g*cos(alpha)/2
        double power = (19.6*Math.cos(alpha))/2;

        telemetry.addData("eh",EA.getCurrentPosition());
        if ((EA.getCurrentPosition() < 1500) && right) {
            EA.setPower(0.4);

        } else if ((EA.getCurrentPosition() > 0) && left) {
            EA.setPower(-0.4);
        }

        else{
            EA.setPower(0.0025);
        }
    }


    public void heightByPress(double right, double left){
        if(right>0 && left>0){
            EH.setPower(0.0005);
        }
        else if ((EH.getCurrentPosition() < 2350) && right > 0) {
            EH.setPower(1);
//            pid_EA.setWanted(EA.getCurrentPosition());
        } else if ((EH.getCurrentPosition() > 0) && left > 0) {
            EH.setPower(-1);
//            pid_EA.setWanted(EA.getCurrentPosition());
        } else {
//            double power_EA = pid_EA.update(EA.getCurrentPosition());
            EH.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            EH.setPower(0.0005);

        }
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
//        double add;
//        if (in){
//            add = 0.01;
//        } else if (out) {
//            add = -0.01;
//        }
//        else{
//            add = 0;
//        }
//        IntakeL.setPosition(IntakeL.getPosition() + add);
//        IntakeR.setPosition(IntakeR.getPosition() + add);
//        telemetry.addData("intakeR",IntakeR.getPosition());
//        telemetry.addData("intakeL",IntakeL.getPosition());
//    }
    public void Change_Angle_Pos() {
        //int count = 0;

//        EH.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        double power;
//        if (count < 3){
//            if (Math.abs(Math.abs(x)-Math.abs(EH.getCurrentPosition())) < thresh){
//                count++;
//            }
        if (Math.abs(intakeWanted - EA.getCurrentPosition()) < thresh) {
            power = 0;
        } else {
            power = pid_EA.update(EA.getCurrentPosition());
        }
        EA.setPower(power);
    }
//    public void Intake_angle(boolean up, boolean down){
//        if (up){intake_center_angle.setPower(1);}
//        else if (down){intake_center_angle.setPower(-1);}
//        else{ intake_center_angle.setPower(0);}
//    }

    public void intakeToPos(){
        double power;
        if (Math.abs(wanted - EH.getCurrentPosition()) < thresh){
            power = 0;
        }
        else{
            power = pid_EH.update(EH.getCurrentPosition());
        }


        EH.setPower(power);


    }

    public void Intake_Angle(boolean up, boolean down){
        double add;
        if (down ){
            add = 0.01;
        } else if (up) {
            add = -0.01;
        }
        else{
            add = 0;
        }
        intake_center_angle.setPosition(intake_center_angle.getPosition() + add);
    telemetry.addData("intake angle",intake_center_angle.getPosition());
    }
    public void preload(){
        Move_Elevator_Angle(1750);
        Move_Elevator(2300);
        intake_center_angle.setPosition(0.57);



    }
}

