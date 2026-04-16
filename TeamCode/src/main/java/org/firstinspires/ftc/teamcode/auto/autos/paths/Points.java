package org.firstinspires.ftc.teamcode.auto.autos.paths;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.PedroCoordinates;
import com.pedropathing.geometry.Pose;
import com.pedropathing.math.MathFunctions;
import com.pedropathing.paths.PathChain;

public class Points {
    // we have commented out code here in case gemini hallucinated adding public Pose
    public static Pose startPose = new Pose(20.2, 122.5, Math.toRadians(144));
    public static Pose startPoseFar = new Pose(56.5, 8.1, Math.toRadians(180));
    public Pose scorePoseFar = new Pose(59, 16, Math.toRadians(112));
    public Pose scorePose = new Pose(47.5, 95, Math.toRadians(135));
    public Pose controlPosePPG = new Pose(70, 60);
    public Pose controlPosePGP = new Pose(58, 58);
    public Pose controlPoseGPP = new Pose(50, 60);
    public Pose controlPoseGate = new Pose(31.5, 57.5);
    public Pose controlPoseGatePPG = new Pose(50, 59);
    public Pose controlPoseGatePickUp = new Pose(50, 59);
    public Pose controlPoseEatLeftoverGate = new Pose(7, -4);
    public Pose leaveClosePose = new Pose(55, 122.5, Math.toRadians(165));
    public Pose leaveFarPose = new Pose(39, 16, Math.toRadians(180));
    public Pose GPP = new Pose(44, 36, Math.toRadians(180));
    public Pose PPG = new Pose(44, 83.5, Math.toRadians(180));
    public Pose PGP = new Pose(44, 58.5, Math.toRadians(180));
    public Pose afterPickupGPP = new Pose(11.5, 36, Math.toRadians(180));
    public Pose afterPickupPPG = new Pose(18, 83.5, Math.toRadians(180));
    public Pose afterPickupPGP = new Pose(11.5, 58.5, Math.toRadians(180));
    public Pose gate = new Pose(18, 67, Math.toRadians(180));
    public Pose pickUpGate = new Pose(10.3, 59.5, Math.toRadians(130));
    public Pose humanPlayer = new Pose(11.5, 8.1, Math.toRadians(180));
    public Pose eatLeftoverGate = new Pose(7.8, 45.5);
    public Pose humanPlayerControlPose = new Pose(30, 18);


//    public Pose gate, pickUpGate, humanPlayer, eatLeftoverGate, humanPlayerControlPose;
//    startPose = new Pose(20.2, 122.5, Math.toRadians(144)); // Start Pose of our robot.
//    startPoseFar = new Pose(56.5, 8.1, Math.toRadians(180)); // Start Pose of our robot.
//    scorePoseFar = new Pose(59, 16, Math.toRadians(112)); // Scoring Pose of our robot. It is facing the goal at a 115 degree angle.
//
//    scorePose = new Pose(47.5, 95, Math.toRadians(135)); // Scoring Pose of our robot. It is facing the goal at a 135 degree angle.
//    controlPosePPG = new Pose(70,60);// pose for getting to PPG without hitting other balls
//    controlPosePGP = new Pose(58,58);// pose for getting to PGP without hitting other balls
//    controlPoseGPP = new Pose(50,60);// pose for getting to PGP without hitting other balls
//    controlPoseGate = new Pose(31.5,57.5);
//
//    controlPoseGatePPG = new Pose(50, 59); // pose for getting from gate to score without hitting ppg
//    controlPoseGatePickUp = new Pose(50, 59); // pose for getting from gate to gate pick up without hitting the gate
//    controlPoseEatLeftoverGate = new Pose(7, -4); // pose for getting from leftover balls from the gate
//
//    leaveClosePose = new Pose(55, 122.5, Math.toRadians(165));
//    leaveFarPose = new Pose(39, 16, Math.toRadians(180));
//    GPP = new Pose(44, 36, Math.toRadians(180));
//    PPG = new Pose(44, 83.5, Math.toRadians(180));
//    PGP = new Pose(44 , 58.5, Math.toRadians(180));
//
//    afterPickupGPP = new Pose(11.5, 36, Math.toRadians(180));
//    afterPickupPPG = new Pose(18, 83.5, Math.toRadians(180));
//    afterPickupPGP = new Pose(11.5, 58.5, Math.toRadians(180));
//    gate = new Pose(18,67,Math.toRadians(180));
//    pickUpGate = new Pose(10.3, 59.5, Math.toRadians(130));
//    humanPlayer = new Pose(11.5,8.1,Math.toRadians(180));
//    eatLeftoverGate = new Pose(7.8,45.5);
//    humanPlayerControlPose = new Pose(30,18);

    public void setPointsToRed() {
        startPose = startPose.mirror(); // Start Pose of our robot.
        startPoseFar = scorePoseFar.mirror();
        scorePoseFar = scorePoseFar.mirror(); // Scoring Pose of our robot. It is facing the goal at a 115 degree angle.
        scorePose = scorePose.mirror(); // Scoring Pose of our robot. It is facing the goal at a 135 degree angle.
        leaveClosePose = leaveClosePose.mirror();
        controlPosePPG = controlPosePPG.mirror();// pose for getting to PPG without hitting other balls
        controlPosePGP = controlPosePGP.mirror();// pose for getting to PGP without hitting other balls
        controlPoseGPP = controlPoseGPP.mirror();// pose for getting to PGP without hitting other balls
        controlPoseGate = controlPoseGate.mirror();
        controlPoseGatePPG = controlPoseGatePPG.mirror();
        controlPoseGatePickUp = controlPoseGatePickUp.mirror();
        GPP = GPP.mirror();
        PPG = PPG.mirror();
        PGP = PGP.mirror();
        afterPickupGPP = afterPickupGPP.mirror();
        afterPickupPPG = afterPickupPPG.mirror();
        afterPickupPGP = afterPickupPGP.mirror();
        leaveFarPose = leaveFarPose.mirror();
        gate = gate.mirror();
        pickUpGate = pickUpGate.mirror();
        humanPlayer = humanPlayer.mirror();
        eatLeftoverGate = eatLeftoverGate.mirror();
        controlPoseEatLeftoverGate = controlPoseEatLeftoverGate.mirror();
        humanPlayerControlPose = humanPlayerControlPose.mirror();


    }


    private Pose Mymirror(Pose pPose) {
        Pose k = pPose;
        return new Pose(141.5 - k.getX(), k.getY()-2, MathFunctions.normalizeAngle(Math.PI - k.getHeading()), PedroCoordinates.INSTANCE);
    }


}