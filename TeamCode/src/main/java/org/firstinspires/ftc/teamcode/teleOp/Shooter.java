package org.firstinspires.ftc.teamcode.teleOp;

import com.acmerobotics.dashboard.config.Config;
import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Misc.GetVelocity;
import org.firstinspires.ftc.teamcode.Misc.PID;


@Configurable
@Config
public class Shooter {
    public DcMotorEx shooter, shooter2;
    Telemetry telemetry;
    GoBildaPinpointDriver odometry;
    public static double kP = 20;
    public static double kI = 0;
    public static double kD = 500;
    public static double kF = 1.14;

    GetVelocity shooterVelocity;
    GetVelocity shooter2Velocity;

    public Shooter(DcMotorEx shootMotor, Telemetry telemetry, DcMotorEx shooter2, GoBildaPinpointDriver odometry){
        this.shooter = shootMotor;
        this.telemetry = telemetry;
        this.shooter2 = shooter2;
        this.odometry = odometry;


        shooterVelocity = new GetVelocity(shooter,0.1);
        shooter2Velocity = new GetVelocity(shooter2,0.1);
    }
    PID controller = new PID(kP,kI,kD,kF);
    public void shoot(double x){
        controller.setWanted(x);
        double output = controller.update(shooterVelocity.getVelocityFilter()/6000);

        shooter.setPower(output);
        shooter2.setPower(-output);
        telemetry.addData("output", output);
    }
    boolean prevMore = false;
    boolean prevLess = false;
    public double power = 0;

    public double getVariableSpeedShoot(boolean more, boolean less, double jumps){

        if(more && !prevMore){power += jumps;}
        else if(less && !prevLess){
            power -= jumps;
        }
        if (power >= 0.7){
            power = 0.7;
        }
        prevLess = less;
        prevMore = more;
        telemetry.addData("wanted variable", power*6000);
        return power;
    }

    public void variableSpeedShoot(boolean more, boolean less, double jumps){
        shoot(getVariableSpeedShoot(more, less, jumps));
    }

    public void variableInterplationSpeedShoot(boolean more, boolean less, double jumps, String Team) {
        shoot(interpolateTel(utils.getDistFromGoal(Team))
                + getVariableSpeedShoot(more, less, jumps));
    }

    public double interpolateTel(double dis){
        return (0.00000149018 * Math.pow(dis, 2) + 0.0000836022 * dis + 0.35805);
    }

    public boolean isUpToGivenSpeed(double speed){
        double threshold = 100; //TODO: tune!! should be the biggest reliably scoring value
        return Math.abs(shooterVelocity.getVelocityFilter() - speed*6000) < threshold;
    }
}