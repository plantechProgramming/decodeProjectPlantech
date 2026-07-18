package org.firstinspires.ftc.teamcode.auto;

import static com.pedropathing.ivy.Scheduler.schedule;

import com.pedropathing.follower.Follower;
import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.Scheduler;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Misc.DataSaving;
import org.firstinspires.ftc.teamcode.Misc.Utils.Alliance;
import org.firstinspires.ftc.teamcode.Misc.Utils.TelemetryUtils;
import org.firstinspires.ftc.teamcode.TeamOpMode;
import org.firstinspires.ftc.teamcode.auto.autos.paths.Paths;
import org.firstinspires.ftc.teamcode.auto.autos.paths.Points;
import org.firstinspires.ftc.teamcode.auto.pedro.Constants;
import org.firstinspires.ftc.teamcode.subsystems.AutoCommands;

public abstract class TeamAuto extends TeamOpMode {
    protected AutoCommands command;
    protected Paths path;
    protected Follower follower;
    protected Boolean isFar;

    @Override
    public abstract void postInit();
    @Override
    public void run(){
        path = new Paths();
        follower = Constants.createFollower(hardwareMap);

        if(isFar){
            follower.setStartingPose(path.points.startPoseFar);
        }
        else{
            follower.setStartingPose(path.points.startPose);
        }
        command = new AutoCommands(follower);
        path.buildPaths(follower);

        schedule(autoRoutine());

        while (opModeIsActive()) {
            TelemetryUtils.updatePedroTelemetry(telemetry, follower);
            TelemetryUtils.updatePedroTelemetry(dashboardTelemetry, follower);

            command.shooter.updateTelemetry(telemetry);
            command.shooter.updateTelemetry(dashboardTelemetry);

            telemetry.update();
            dashboardTelemetry.update();
            DataSaving.setEndPos(follower.getPose());

            schedule(command.periodic());
            Scheduler.execute();
            follower.update();
        }
    }

    public abstract Command autoRoutine();

    @Override
    protected void end() {
    }
}
