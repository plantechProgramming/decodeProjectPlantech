package org.firstinspires.ftc.teamcode.Misc.Utils;

import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.Misc.InitMotors;
import org.firstinspires.ftc.teamcode.Misc.Utils.filters.LowPass;

public class Extras {
    LowPass lowPass = new LowPass();
    public double getVoltageCompensatedPow(double pow, double voltage){
        lowPass.start(0.02);
        lowPass.update(voltage);
        return pow * (14/lowPass.get());
    }

}
