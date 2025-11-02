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
    private ServoEx hood = new ServoEx("hood");

    // g - gravity acceleration
    final double g = 9.8066;
    // h - goal height + some 5 cm. IN CM
    final double h = 1.15;
    final double diameter = .096; //in mm
    final int MAX_RPM = 6000;
    final double robot_height = 0.4;
    double lastDistance = 1.5; //TODO: make depend on actual starting pos
    public final Position CAM_POS = new Position(DistanceUnit.CM,
            0, 0, 0, 0);
    private final YawPitchRollAngles CAM_ORIENTATION = new YawPitchRollAngles(AngleUnit.DEGREES,0,-90,0,0);
    AprilTagLocalization test = new AprilTagLocalization();
    AprilTagProcessor aprilTag = new AprilTagProcessor.Builder()
            .setCameraPose(CAM_POS, CAM_ORIENTATION)
            .setTagFamily(AprilTagProcessor.TagFamily.TAG_36h11)
            .setTagLibrary(AprilTagGameDatabase.getCurrentGameTagLibrary())
            .setOutputUnits(DistanceUnit.METER, AngleUnit.DEGREES)
            .build();

    VisionPortal.Builder builder = new VisionPortal.Builder();
    public Command shootByAngle() {
        test.detectTags(aprilTag);
        double d = 0;
        try {
            AprilTagDetection goalTag = test.specialDetection;
            d = test.distanceToGoal(goalTag.robotPose,goalTag.id);
            lastDistance = d;
        } catch (NullPointerException e){
            d = lastDistance;
        }

        double theta = 0.804; // in radians
        double t = Math.sqrt((2 / g) * (Math.tan(theta) * d - (h - robot_height)));
        double velocity = 2 * d / (Math.cos(theta) * t);
        double motorPower = 60 * velocity / (diameter * Math.PI * MAX_RPM);

        return new ParallelGroup(
                new SetPower(shooter1,1),
                new SetPower(shooter2,-1)
        );

    }
}
