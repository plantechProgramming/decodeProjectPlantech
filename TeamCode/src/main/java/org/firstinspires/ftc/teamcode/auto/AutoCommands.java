package org.firstinspires.ftc.teamcode.auto;


import com.rowanmcalpin.nextftc.core.command.Command;
import com.rowanmcalpin.nextftc.core.command.groups.ParallelGroup;
import com.rowanmcalpin.nextftc.core.command.groups.SequentialGroup;
import com.rowanmcalpin.nextftc.ftc.NextFTCOpMode;

import org.firstinspires.ftc.teamcode.auto.subsystems.ElevatorAngleNext;
import org.firstinspires.ftc.teamcode.auto.subsystems.intake.nextIntakeAngle;
import org.firstinspires.ftc.teamcode.auto.subsystems.intake.nextIntakeClaw;
import org.firstinspires.ftc.teamcode.auto.subsystems.nextLift;

public class AutoCommands extends NextFTCOpMode{
//    Elevator elevator = new Elevator(DcMotor EA, );

    public AutoCommands() {
        super(nextLift.INSTANCE, nextIntakeAngle.INSTANCE, ElevatorAngleNext.INSTANCE,nextIntakeClaw.INSTANCE);
    }
    nextLift lift = nextLift.INSTANCE;
    nextIntakeAngle intakeAngle = nextIntakeAngle.INSTANCE;
    ElevatorAngleNext elevatorAngle = ElevatorAngleNext.INSTANCE;
    nextIntakeClaw intakeClaw = nextIntakeClaw.INSTANCE;


    public Command takeSample(){
        return new SequentialGroup(
                new ParallelGroup(
                        lift.toHeight(2400,0.5),
                        intakeAngle.Down(),
                        intakeClaw.out(2)
                ),
                intakeClaw.in(0.5)
        );
    }
    public Command sampleToBasket(){
        return new SequentialGroup(


                elevatorAngle.toAngle(1700, 1),
                lift.toHeight(2300,2),
            intakeAngle.Up(),
            intakeClaw.out(2),



                lift.toHeight(0,0.5),
                elevatorAngle.toAngle(0, 1)

             );
    }

    // not very useful



    @Override
    public void onStartButtonPressed() {

    }
}


