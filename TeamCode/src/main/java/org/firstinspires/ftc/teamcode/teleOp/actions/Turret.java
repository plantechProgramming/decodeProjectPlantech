package org.firstinspires.ftc.teamcode.teleOp.actions;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import org.firstinspires.ftc.robotcore.external.Telemetry;
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
        TICK_PER_TURN = 383.6;
        TICK_PER_DEG = TICK_PER_TURN/360;
        this.odometry = odometry;
    }

    // turns to deg and accounts for robot turning
    // when the robot turns, the measured turret angle doesnt change according to its turns
    // so you need to add the diff in heading to the turret heading every time
    // so the actual pos ISNT THE ENCODER POS and we CANT USE the built in pid

    boolean isCableStretched = false;
    double power;
    double newdeg;
    public void turnToDegCorrected(double deg){
        double currentDeg = getRealDeg()*GEAR_RATIO; // angle in motor
        PID pid = new PID(0.007 , 0.00000004, 0.00002, 0); // kp = 0.008, ki = 0.00000006, kd = 0.00002, kf = 0//
        double threshold = 1;
        newdeg = deg * GEAR_RATIO; // angle in motor
        pid.setWanted(newdeg);

        if(Math.abs(utils.getDistBetweenAngles(newdeg, currentDeg)) > threshold){ // if not in threshold
            if(!isCableStretched){
                isCableStretched = isCableStretched(deg);
            }
//            power = pid.updateTurretDeg(currentDeg, GEAR_RATIO, isCableStretched);
            power = pid.update(currentDeg);
            turretMotor.setPower(power);
        }
        else{
            isCableStretched = false;
        }
    }
    public double convertMotorAxisToRobot(double angle){
        if(angle>180 && angle < 360){
            return angle - 360;
        }
        return angle;
    }
    double maxPos = 166; // margin of error = 7 deg
    double minPos = -146;
    double cableZero = 78.3;
    double actualWanted;
    double actualCableZero;
    public boolean isCableStretched(double wanted){
//        if((actualWanted - cableZero) > maxPos || -(actualWanted - cableZero) < minPos){
//        actualWanted = convertDegToReal(wanted);
        actualWanted = wanted;
        actualCableZero = convertDegToReal(cableZero);
        if(utils.getDistBetweenAngles(actualWanted, actualCableZero) > maxPos || utils.getDistBetweenAngles(actualWanted, actualCableZero) < minPos){
            return true;
        }
        return false;
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

    public double convertDegToReal(double deg){
        odometry.update();
        return deg + odometry.getHeading(AngleUnit.DEGREES);
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

    public void setTelemetry(Telemetry telemetry){
        telemetry.addData("dist from cable zero",utils.getDistBetweenAngles(actualWanted, actualCableZero));
        telemetry.addData("turret deg (corrected)", this.getRealDeg());
        telemetry.addData("turret pow", power);
        telemetry.addData("is stretched?", isCableStretched);
        telemetry.addData("wanted", newdeg/GEAR_RATIO);
    }
}
