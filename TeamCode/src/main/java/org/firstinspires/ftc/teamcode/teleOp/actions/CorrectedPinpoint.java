package org.firstinspires.ftc.teamcode.teleOp.actions;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.I2cDeviceSynchSimple;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class CorrectedPinpoint extends GoBildaPinpointDriver {
    public CorrectedPinpoint(I2cDeviceSynchSimple deviceClient, boolean deviceClientIsOwned) {
        super(deviceClient, deviceClientIsOwned);
    }
    private static final float goBILDA_SWINGARM_POD = 13.26291192f; //ticks-per-mm for the goBILDA Swingarm Pod
    private static final float goBILDA_4_BAR_POD = 19.89436789f;
    final float YToXRatio = goBILDA_4_BAR_POD/goBILDA_SWINGARM_POD;

    @Override
    public double getPosY(DistanceUnit distanceUnit){
        return getEncoderY()*YToXRatio;
    }

}
