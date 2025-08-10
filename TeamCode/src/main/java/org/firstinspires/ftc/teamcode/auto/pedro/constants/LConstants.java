package org.firstinspires.ftc.teamcode.auto.pedro.constants;

import com.pedropathing.localization.Encoder;
import com.pedropathing.localization.constants.TwoWheelConstants;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;

public class LConstants {
    static {
        TwoWheelConstants.forwardTicksToInches = 0.003;
        TwoWheelConstants.strafeTicksToInches = 0.00196721068372003;
        TwoWheelConstants.forwardY = -18.25;
        TwoWheelConstants.strafeX = 12.1;
        TwoWheelConstants.forwardEncoder_HardwareMapName = "FR";
        TwoWheelConstants.strafeEncoder_HardwareMapName = "BL";
        TwoWheelConstants.forwardEncoderDirection = Encoder.FORWARD;
        TwoWheelConstants.strafeEncoderDirection = Encoder.REVERSE;
        TwoWheelConstants.IMU_HardwareMapName = "imu";
        TwoWheelConstants.IMU_Orientation = new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.UP, RevHubOrientationOnRobot.UsbFacingDirection.FORWARD);
    }
}




