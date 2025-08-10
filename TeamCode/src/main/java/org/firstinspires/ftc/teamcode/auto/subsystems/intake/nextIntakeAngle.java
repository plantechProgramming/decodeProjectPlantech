package org.firstinspires.ftc.teamcode.auto.subsystems.intake;


import com.qualcomm.robotcore.hardware.Servo;
import com.rowanmcalpin.nextftc.core.Subsystem;
import com.rowanmcalpin.nextftc.core.command.Command;
import com.rowanmcalpin.nextftc.core.control.controllers.PIDFController;
import com.rowanmcalpin.nextftc.core.control.controllers.feedforward.StaticFeedforward;
import com.rowanmcalpin.nextftc.ftc.OpModeData;
import com.rowanmcalpin.nextftc.ftc.hardware.ServoToPosition;

public class nextIntakeAngle extends Subsystem {
    // BOILERPLATE
    public static final nextIntakeAngle INSTANCE = new nextIntakeAngle();
    public nextIntakeAngle() { }
    public Servo intA;

    // same as TeleOp roni2_intake
    public Servo clawAngle;
    public String intake_center_angle = "intA";
    public String claw = "intake";
    public PIDFController PID_intA = new PIDFController(0.005, 0, 0, new StaticFeedforward(0));



    public Command Up() {
        return new ServoToPosition(intA, // SERVO TO MOVE
                0.9,// POSITION TO MOVE TO
                this); // IMPLEMENTED SUBSYSTEM
    }

    public Command Down() {
        return new ServoToPosition(intA, // SERVO TO MOVE
                0.5, // POSITION TO MOVE TO
                this); // IMPLEMENTED SUBSYSTEM
    }

    @Override
    public void initialize() {
        intA = OpModeData.INSTANCE.getHardwareMap().get(Servo.class, intake_center_angle);
        intA.setPosition(0.7);
    }

}


