package org.firstinspires.ftc.teamcode.auto.subsystems;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.auto.camera.AprilTagLocalization;
import org.firstinspires.ftc.teamcode.teleOp.actions.Shooter;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagGameDatabase;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.groups.ParallelGroup;
import dev.nextftc.core.commands.groups.SequentialGroup;
import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.core.units.Distance;
import dev.nextftc.hardware.impl.MotorEx;
import dev.nextftc.hardware.impl.ServoEx;
import dev.nextftc.hardware.positionable.SetPosition;
import dev.nextftc.hardware.powerable.SetPower;

public class NextShooter implements Subsystem {
    public static final NextShooter INSTANCE = new NextShooter();
    public NextShooter() { }
    private MotorEx shooter1 = new MotorEx("shooter");
    private MotorEx shooter2 = new MotorEx("shooter2");
    double Szonedis = 0.55;
    double errorFix = 1.19;

    public Command naiveShooter(double dis) {
        if (dis <= 1.3) {
            Szonedis = .48;
            return new ParallelGroup(new SetPower(shooter1, Szonedis*errorFix),
                new SetPower(shooter2, -Szonedis*errorFix));
        } else {
            Szonedis = 0.55;
            return new ParallelGroup(new SetPower(shooter1, Szonedis*errorFix),
                    new SetPower(shooter2, -Szonedis*errorFix));
        }
    }

}
