package org.firstinspires.ftc.teamcode.Misc.Utils.filters;

import com.google.firebase.crashlytics.buildtools.ndk.internal.dwarf.processor.CompilationUnitContext;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

public class PoseLowPass extends Filter<Pose2D> {
    LowPass xlowPass;
    LowPass ylowPass;
    AngleLowPass angleLowPass;
    public PoseLowPass(double XYAlpha, double headingAlpha){
        xlowPass = new LowPass(XYAlpha);
        ylowPass = new LowPass(XYAlpha);
        angleLowPass = new AngleLowPass(headingAlpha);
        filtered = new Pose2D(DistanceUnit.CM, 0, 0, AngleUnit.DEGREES, 0);
    }

    @Override
    public void update(Pose2D curr) {
        xlowPass.update(curr.getX(DistanceUnit.CM));
        ylowPass.update(curr.getY(DistanceUnit.CM));
        angleLowPass.update(curr.getHeading(AngleUnit.DEGREES));
        filtered = new Pose2D(DistanceUnit.CM, xlowPass.get(), ylowPass.get(),
                AngleUnit.DEGREES, angleLowPass.get());
    }

    @Override
    public void reset() {
        xlowPass.reset();
        ylowPass.reset();
        angleLowPass.reset();
    }
}
