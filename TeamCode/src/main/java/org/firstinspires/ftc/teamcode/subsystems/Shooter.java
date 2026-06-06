package org.firstinspires.ftc.teamcode.subsystems;

import static com.pedropathing.ivy.groups.Groups.parallel;

import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.commands.Commands;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Misc.Alliance;
import org.firstinspires.ftc.teamcode.Misc.PID;
import org.firstinspires.ftc.teamcode.Misc.GetVelocity;
import org.firstinspires.ftc.teamcode.Misc.Utils.TelemetryUtils;

public class Shooter {
    DcMotorEx shootMotor, shootMotorOp;
    GetVelocity shooterVel;
    double MAX_RPM = 6000;
    public Shooter(HardwareMap hardwareMap) {
        initHardware(hardwareMap);
        shooterVel = new GetVelocity(shootMotor, 0.1);
    }

    private void initHardware(HardwareMap hardwareMap){
        shootMotor = hardwareMap.get(DcMotorEx.class, "shooter");
        shootMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        shootMotor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);

        shootMotorOp = hardwareMap.get(DcMotorEx.class, "shooter2");
        shootMotorOp.setDirection(DcMotorSimple.Direction.FORWARD);
        shootMotorOp.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
    }

    double wantedPow;
    public static double farPow = 0.541;
    public static double closePow = 0.387;

    public static double kP = 20;
    public static double kI = 0;
    public static double kD = 500;
    public static double kF = 1.14;

    // TODO: tune
    PID controller = new PID(kP,kI,kD,kF);

    public Command naiveShooter(boolean far) {
        if (far) {
            wantedPow = farPow;
        } else {
            wantedPow = closePow;
        }
        return Commands.instant(()->controller.setWanted(wantedPow));
    }

    public void updateTelemetry(Telemetry telemetry){
        TelemetryUtils.addTitle(telemetry, "staring shooter telemetry");
        telemetry.addData("current pow",getCurPower());
        TelemetryUtils.addVar(telemetry, wantedPow);
        telemetry.addData("wanted v", wantedPow*MAX_RPM);
        telemetry.addData("cur v", shooterVel.getVelocityFilter());
        TelemetryUtils.addTitle(telemetry, "ending shooter telemetry");
    }

    public Command setShooterPowerAsCommand(double pow){
        return parallel(
                Commands.instant(()->shootMotor.setPower(pow)),
                Commands.instant(()->shootMotorOp.setPower(-pow))
        );
    }
    public double getCurPower(){
        return shooterVel.getVelocityFilter()/MAX_RPM;
    }

    public void periodic(){
        double output = controller.update(getCurPower());
//        return infinite(() -> {setPower(output);});
        setPower(output);
    }

    public void setPower(double pow){
        shootMotor.setPower(pow);
        shootMotorOp.setPower(-pow);
    }

    public void out(){
        setPower(-0.2);
    }

    public double getInterpolation(double dis){
        return (0.00000149018 * Math.pow(dis, 2) + 0.0000836022 * dis + 0.35805);
    }

    public void variableShoot(boolean more, boolean less, double jumps){
        controller.setWanted(getVariableShoot(more, less, jumps));
    }

    public boolean isUpToGivenSpeed(double wantedSpeed, double curSpeed){
        double threshold = 100; //TODO: tune!! should be the biggest reliably scoring value
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

    public double getInterplationVariableShoot(boolean more, boolean less, double jumps, Alliance team) {
        return getInterpolation(utils.getDistFromGoal(team))
                + getVariableShoot(more, less, jumps);
    }

    public void interplationVariableShoot(boolean more, boolean less, double jumps, Alliance team) {
        controller.setWanted(getInterplationVariableShoot(more, less, jumps, team));
    }
}
