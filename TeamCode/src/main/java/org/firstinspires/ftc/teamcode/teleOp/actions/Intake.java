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

