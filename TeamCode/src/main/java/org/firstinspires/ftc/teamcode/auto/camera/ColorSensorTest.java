package org.firstinspires.ftc.teamcode.auto.camera;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class ColorSensorTest {
    NormalizedColorSensor colorSensor;
//    public boolean

    public enum DetectedColor{
        GREEN,
        BLUE,
        RED,
        PURPLE,

        UNKNOWN
    }
    public void init(HardwareMap hawamap){
        colorSensor = hawamap.get(NormalizedColorSensor.class, "color_sensor");
        colorSensor.setGain(8);
    }
    public DetectedColor getDetectedColor(Telemetry telemetry){
        NormalizedRGBA colors = colorSensor.getNormalizedColors();// returns RGB and Alpha values

        float normRed,normGreen,normBlue;
        normRed = colors.red/ colors.alpha;
        normGreen = colors.green/ colors.alpha;
        normBlue = colors.blue/ colors.alpha;

        //TODO add the if statements for specific colors
        /*
            red green blue
        Green:<.5,.9>,<.4
        Red:>.4,<.32,<.15
        Blue: <.1, <.25, >.55

         */
        telemetry.addData("red: ",normRed);
        telemetry.addData("green: ", normGreen);
        telemetry.addData("blue: ",normBlue);
        if (normRed>0.19 && normGreen<.37 && normBlue<.3){
            return DetectedColor.RED;
        }else if (normRed > 0.14 && normGreen>0.2 && normBlue<0.4){
            return DetectedColor.GREEN;
        }else if (normRed < 0.2 && normGreen<0.5 && normBlue>0.20){
            return DetectedColor.BLUE;
        }else {
            return DetectedColor.UNKNOWN;
        }
    }
}
