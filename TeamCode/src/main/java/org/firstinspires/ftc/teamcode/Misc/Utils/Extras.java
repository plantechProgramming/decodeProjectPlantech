package org.firstinspires.ftc.teamcode.Misc.Utils;

import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.InitMotors;

public class Extras {

    public static void setDriveToBrakeMode(){
        InitMotors.BL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        InitMotors.BR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        InitMotors.FR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        InitMotors.FL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public static void setDriveToFloatMode(){
        InitMotors.BL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        InitMotors.BR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        InitMotors.FR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        InitMotors.FL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
    }

}
