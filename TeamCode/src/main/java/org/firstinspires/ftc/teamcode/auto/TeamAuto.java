package org.firstinspires.ftc.teamcode.auto;

import com.pedropathing.follower.Follower;
import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.Scheduler;

import org.firstinspires.ftc.teamcode.Misc.DataSaving;
import org.firstinspires.ftc.teamcode.Misc.Utils.Alliance;
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
            follower.setStartingPose(Points.startPoseFar);
        }
        else{
            follower.setStartingPose(Points.startPose);
        }
        command = new AutoCommands(follower);
        path.buildPaths(follower);

        Scheduler.schedule(autoRoutine());

        while (opModeIsActive()) {
            command.periodic();
            Scheduler.execute();
        }
    }

    public abstract Command autoRoutine();

    @Override
    protected void end() {
        DataSaving.setEndPos(follower.getPose());
    }
}
