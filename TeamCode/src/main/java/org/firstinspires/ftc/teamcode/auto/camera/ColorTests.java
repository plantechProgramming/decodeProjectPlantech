package org.firstinspires.ftc.teamcode.auto.camera;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.HardwareMap;

@TeleOp
public class ColorTests extends OpMode {
    ColorSensorTest color = new ColorSensorTest();
    ColorSensorTest.DetectedColor detectedColor;
    public boolean colormatch;

    @Override
    public void init() {
        color.init(hardwareMap);
    }

    @Override
    public void loop() {
        detectedColor = color.getDetectedColor(telemetry);
        telemetry.addData("Detected Color is: ", detectedColor);
    }
}
