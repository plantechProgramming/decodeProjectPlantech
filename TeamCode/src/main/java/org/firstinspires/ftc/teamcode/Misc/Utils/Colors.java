package org.firstinspires.ftc.teamcode.Misc.Utils;

import org.opencv.core.Scalar;

public class Colors {
    private Scalar min;
    private Scalar max;
    public static final Colors RED = new Colors(
            new Scalar(112, 153, 230),
            new Scalar(190, 230, 255)
    );

    public static final Colors YELLOW = new Colors(
            new Scalar(16, 150, 99),
            new Scalar(40, 255, 255)
    );

    // yellow - in opencv hsv
    // opencv hsv != normal hsv, h:0-179, s:0-255,v:0-255
    public Colors(Scalar min, Scalar max){
        this.min = min;
        this.max = max;
    }

    public Scalar getMax() {
        return max;
    }

    public Scalar getMin() {
        return min;
    }
}
