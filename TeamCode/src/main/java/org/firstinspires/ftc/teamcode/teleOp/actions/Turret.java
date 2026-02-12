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
    double currentDeg;
    double currentError;
    public void turnToDegCorrected(double deg){
        currentDeg = getRealDeg(); // angle in turret
        PID pid = new PID(0.007 , 0.00000004, 0.00002, 0); // kp = 0.007, ki = 0.00000004, kd = 0.00002, kf = 0//
        double threshold = 1.5;
        newdeg = deg; // angle in turret
        pid.setWanted(newdeg);

        if(Math.abs(utils.getDiffBetweenAngles(newdeg, currentDeg)) > threshold){ // if not in threshold
            if(!isCableStretched){
                isCableStretched = isCableStretched(getCurDeg());
            }
            power = pid.updateTurretDeg(currentDeg, isCableStretched);
//            power = pid.update(currentDeg);
            turretMotor.setPower(power);
            //TEMP
            currentError = utils.getDiffBetweenAngles(newdeg, currentDeg);
            if(isCableStretched){
                if(currentError > 0){
                    currentError = 360 - currentError;
                }
                else{
                    currentError += 360;
                }
            }

        }
        else{
            isCableStretched = false;
            turretMotor.setPower(0); // because the motor remembers the last pow
        }
    }
    public double convertMotorAxisToRobot(double angle){
        if(angle>180 && angle < 360){
            return angle - 360;
        }
        return angle;
    }

    double cableZero = 78.3;// in gyro coordinate system
    double maxDiff = 225 - cableZero; // 225 in turret cords infinite to -infinite
    double minDiff = -140 - cableZero;
    double actualWanted;
    double actualCableZero;
    public boolean isCableStretched(double wanted){
        return !(minDiff-cableZero <= wanted && wanted <= maxDiff-cableZero);
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
//        telemetry.addData("dist from cable zero",actualWanted - actualCableZero);
//        telemetry.addData("actualWanted",actualWanted);
//        telemetry.addData("actualcablezero",actualCableZero);
        telemetry.addData("turret deg (corrected)", this.getRealDeg());
        telemetry.addData("turret pow", power);
        telemetry.addData("is stretched?", isCableStretched(getCurDeg()));
        telemetry.addData("wanted", newdeg);
        telemetry.addData("error", utils.getDiffBetweenAngles(newdeg, currentDeg));
        telemetry.addData("curr deg", getCurDeg());

//        double er = utils.getDiffBetweenAngles(-45, getRealDeg());
//        telemetry.addData("Start errorGood",er);
//        if(er < 0){
//            er += 360;
//        }
//        telemetry.addData("errorGood",er);
//        telemetry.addData("currentDeg", getCurDeg());
    }
}
