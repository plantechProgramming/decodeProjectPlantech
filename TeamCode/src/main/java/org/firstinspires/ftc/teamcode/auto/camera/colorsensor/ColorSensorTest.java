package org.firstinspires.ftc.teamcode.auto.camera.colorsensor;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class ColorSensorTest {
    NormalizedColorSensor colorSensor;
//    public boolean

    public enum DetectedColor{
        GREEN,
        PURPLE,
        UNKNOWN
    }
    public void init(HardwareMap hawamap){
        colorSensor = hawamap.get(NormalizedColorSensor.class, "color_sensor");
        colorSensor.setGain(10);
    }
    public DetectedColor getDetectedColor(Telemetry telemetry){
        NormalizedRGBA colors = colorSensor.getNormalizedColors();// returns RGB and Alpha values

        float normRed,normGreen,normBlue;
        normRed = colors.red/ colors.alpha;
        normGreen = colors.green/ colors.alpha;
        normBlue = colors.blue/ colors.alpha;


        /*
            red green blue
        Green:<.5,.9>,<.4
        Red:>.4,<.32,<.15
        Blue: <.1, <.25, >.55

         */

        telemetry.addData("red: ",normRed);
        telemetry.addData("green: ", normGreen);
        telemetry.addData("blue: ",normBlue);
        if (normRed <.16 && normGreen>0.36 && normBlue<0.53){
            return DetectedColor.GREEN;
        }
        else if(normRed>.11 &&normGreen<.33 && normBlue>.18){
            return DetectedColor.PURPLE;
        }
        else {
            return DetectedColor.UNKNOWN;
        }
    }
}
