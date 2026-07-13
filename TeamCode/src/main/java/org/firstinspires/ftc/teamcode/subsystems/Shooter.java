package org.firstinspires.ftc.teamcode.subsystems;

import static com.pedropathing.ivy.groups.Groups.parallel;

import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.commands.Commands;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Misc.InitMotors;
import org.firstinspires.ftc.teamcode.Misc.PID;
import org.firstinspires.ftc.teamcode.Misc.GetVelocity;
import org.firstinspires.ftc.teamcode.Misc.RobotPose;
import org.firstinspires.ftc.teamcode.Misc.Utils.PoseFunctions;
import org.firstinspires.ftc.teamcode.Misc.Utils.TelemetryUtils;

public class Shooter {
    DcMotorEx shootMotor, shootMotorOp;
    GetVelocity shooterVel;

    PoseFunctions poseFuncs;
    double MAX_RPM = 6000;
    public Shooter() {
        shootMotor = InitMotors.shootMotor;
        shootMotorOp = InitMotors.shootMotorOp;
        shooterVel = new GetVelocity(shootMotor, 0.1);
        poseFuncs = new PoseFunctions(new RobotPose(InitMotors.odometry));
    }

    double wantedNaivePow = 0;
    public static double farPow = 0.541;
    public static double closePow = 0.387;

    public static double kP = 5;
    public static double kI = 0;
    public static double kD = 0;
    public static double kF = 1.14;

    PID controller = new PID(kP,kI,kD,kF);

    public Command naiveShooter(boolean far) {
        if (far) {
            wantedNaivePow = farPow;
        } else {
            wantedNaivePow = closePow;
        }
        return Commands.instant(()->controller.setWanted(wantedNaivePow));
    }

    public void updateTelemetry(Telemetry telemetry){
        TelemetryUtils.addTitle(telemetry, "staring shooter telemetry");
        telemetry.addData("current pow",getCurPower());
        telemetry.addData("cur v", shooterVel.getRawVelocity());
        telemetry.addData("wanted naive vel", wantedNaivePow*MAX_RPM);
        telemetry.addData("wanted interpolation vel", wantedVariableInterpolation*MAX_RPM);
        telemetry.addData("wanted variable vel", variablePower*MAX_RPM);
        telemetry.addData("controller wanted", controller.wanted);
        telemetry.addData("controller pow", controller.update(getCurPower()));
        TelemetryUtils.addTitle(telemetry, "ending shooter telemetry");
    }

    public Command setShooterPowerAsCommand(double pow){
        return parallel(
                Commands.instant(()->shootMotor.setPower(pow)),
                Commands.instant(()->shootMotorOp.setPower(-pow))
        );
    }
    public double getCurPower(){
        return shooterVel.getRawVelocity()/MAX_RPM;
    }

    public void periodic(){
        setPower(controller.update(getCurPower()));
    }

    public void setPower(double pow){
        shootMotor.setPower(pow);
        shootMotorOp.setPower(-pow);
    }

    public Command out(){
        return setShooterPowerAsCommand(-0.2);
    }

    public boolean isUpToGivenSpeed(double wantedSpeed, double curSpeed){
        double threshold = 100; // should be the biggest reliably scoring value
        return Math.abs(curSpeed - wantedSpeed*6000) < threshold;
    }

    boolean prevMore = false;
    boolean prevLess = false;
    double variablePower = 0;
    public double getVariableShoot(boolean more, boolean less, double jumps){
        if(more && !prevMore){variablePower += jumps;}
        else if(less && !prevLess){
            variablePower -= jumps;
        }
        if (variablePower >= 0.7){
            variablePower = 0.7;
        }
        prevLess = less;
        prevMore = more;
        return variablePower;
    }

    public void variableShoot(boolean more, boolean less, double jumps){
        controller.setWanted(getVariableShoot(more, less, jumps));
    }

    public double getInterpolation(double dis){
        return (0.00000149018 * Math.pow(dis, 2) + 0.0000836022 * dis + 0.35805);
    }

    double wantedVariableInterpolation = 0;
    public double getInterpolationVariableShoot(boolean more, boolean less, double jumps) {
        wantedVariableInterpolation = getInterpolation(poseFuncs.getDistFromGoal()) + getVariableShoot(more, less, jumps);
        return wantedVariableInterpolation;
    }

    public void interpolationVariableShoot(boolean more, boolean less, double jumps) {
        controller.setWanted(getInterpolationVariableShoot(more, less, jumps));
    }
}
