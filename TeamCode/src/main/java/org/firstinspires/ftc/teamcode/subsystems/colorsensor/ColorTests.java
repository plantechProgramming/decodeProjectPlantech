package org.firstinspires.ftc.teamcode.subsystems.colorsensor;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

public class ColorTests extends OpMode {
    ColorSensor color = new ColorSensor();
    ColorSensor.DetectedColor detectedColor;
//    public boolean colormatch;

    @Override
    public void init() {
        color.init(hardwareMap);
    }

    @Override
    public void loop() {
        detectedColor = color.getDetectedColor(telemetry);
        telemetry.addData("Detected Color is: ", detectedColor);
        telemetry.addData("last Detected Color: ", detectedColor);
    }
}
