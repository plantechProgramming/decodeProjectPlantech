package org.firstinspires.ftc.teamcode.teleOp.actions;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.teleOp.PID;
import org.firstinspires.ftc.teamcode.teleOp.Utils;

public class Turret {
    public DcMotorEx turretMotor;
    private final double GEAR_RATIO = 2.5; // TODO: update for final version
    private final double TICK_PER_TURN; // TODO: check!!
    private final double TICK_PER_DEG;
    private final Utils utils = new Utils();
    private final GoBildaPinpointDriver odometry;
    public Turret(DcMotorEx turretMotor, GoBildaPinpointDriver odometry){
        this.turretMotor = turretMotor;
        TICK_PER_TURN = 1425.2;
        TICK_PER_DEG = TICK_PER_TURN/360;
        this.odometry = odometry;
    }

    // turns to deg and accounts for robot turning
    // when the robot turns, the measured turret angle doesnt change according to its turns
    // so you need to add the diff in heading to the turret heading every time
    // so the actual pos ISNT THE ENCODER POS and we CANT USE the built in pid

    public void turnToDegCorrected(double deg){
        double currentDeg = getRealDeg()*GEAR_RATIO;
        PID pid = new PID(0.02, 0, 0, 0);
        double threshold = 1;
        double power;
        double newdeg = deg * GEAR_RATIO;
        pid.setWanted(newdeg);

        if(Math.abs(newdeg - currentDeg) > threshold){ // if not in threshold
            power = pid.update(currentDeg);
            turretMotor.setPower(power);
        }
    }
    public double convertMotorAxisToRobot(double angle){
        if(angle>180 && angle < 360){
            return angle - 360;
        }
        return angle;
    }
    public double convertModuloPos(double angle){
        return convertMotorAxisToRobot(utils.angleModulo(angle));
    }
    public double getRealDeg(){
        odometry.update();
        double convertedAngle = convertModuloPos(this.getCurDeg());
        double realAngle = convertedAngle + odometry.getHeading(AngleUnit.DEGREES);
        return  convertModuloPos(realAngle);
    }
    public double getCurDeg(){
        double deg = turretMotor.getCurrentPosition()/TICK_PER_DEG;
        return deg/GEAR_RATIO;
    }
    public double getRev(){
        return turretMotor.getCurrentPosition()/TICK_PER_TURN * GEAR_RATIO;
    }

    public int degToTicks(double deg){
        double ticks = deg * TICK_PER_DEG;
        return (int)Math.round(ticks*GEAR_RATIO); // FUCK JAVA
    }
    public void turnToDeg(double deg){
//        PIDFCoefficients pidf = new PIDFCoefficients(10,0,0,0);
//        turretMotor.setPIDFCoefficients(DcMotor.RunMode.RUN_TO_POSITION, pidf);
        turretMotor.setTargetPosition(degToTicks(deg));
        turretMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        turretMotor.setVelocity(turretMotor.getMotorType().getAchieveableMaxTicksPerSecond());
    }

}
