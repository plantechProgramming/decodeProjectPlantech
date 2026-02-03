package org.firstinspires.ftc.teamcode.auto.subsystems;

import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cCompassSensor;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

import dev.nextftc.control.ControlSystem;
import dev.nextftc.control.KineticState;
import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.delays.Delay;
import dev.nextftc.core.commands.groups.SequentialGroup;
import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.hardware.controllable.RunToPosition;
import dev.nextftc.hardware.impl.MotorEx;
import dev.nextftc.hardware.positionable.SetPosition;
import dev.nextftc.hardware.powerable.SetPower;

public class NextTurret implements Subsystem {
    private MotorEx turretMotor = new MotorEx("",-1);//todo: put motor name when in config
    private final double GEAR_RATIO = 2.5;
    private final double TICK_PER_TURN = 1425.2;
    private final double TICK_PER_DEG = TICK_PER_TURN/360;
    private double kp=0.01,ki=0,kd=0,kf=0.012;//todo - change when turret is real
    ControlSystem controlSystem = ControlSystem.builder()
            .velPid(kp,ki,kd)
            .basicFF(0,0,kf)
            .build();
    public NextTurret(){}
    public int degToTicks(double deg){
        double ticks = deg * TICK_PER_DEG;
        return (int)Math.round(ticks*GEAR_RATIO); // I ❤ JAVA AND GIT
    }
    public Command turnToDeg(double deg){
        return new RunToPosition(controlSystem,degToTicks(deg));
    }
    public void stop(){
        turretMotor.setPower(0);
    }
    public Pose2D PedroPoseConverter(Pose pose){
        double x = pose.getX();
        double y = pose.getY();
        double hed = Math.toDegrees(pose.getHeading());
        double lenField = 365.76; // 144 inch to cm
        double newx = ((-lenField/144)*x)+lenField/2;
        double newy = ((-lenField/144)*y)+lenField/2;
        hed = hed - 180;
        if(hed <= 180){
            hed += 360;
        }
        return new Pose2D(DistanceUnit.CM, newx, newy, AngleUnit.DEGREES, hed);
    }
}
