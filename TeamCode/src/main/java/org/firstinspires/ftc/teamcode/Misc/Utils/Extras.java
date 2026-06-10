package org.firstinspires.ftc.teamcode.Misc.Utils;

import android.util.Pair;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.InitMotors;
import org.firstinspires.ftc.teamcode.Misc.Alliance;
import org.firstinspires.ftc.teamcode.auto.AutoCommands;
import org.firstinspires.ftc.teamcode.auto.autos.paths.Paths;
import org.firstinspires.ftc.teamcode.auto.autos.paths.Points;
import org.firstinspires.ftc.teamcode.auto.pedro.Constants;

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
