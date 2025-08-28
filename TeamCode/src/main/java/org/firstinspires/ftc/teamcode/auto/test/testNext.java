package org.firstinspires.ftc.teamcode.auto.test;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.rowanmcalpin.nextftc.core.command.Command;
import com.rowanmcalpin.nextftc.core.command.groups.ParallelDeadlineGroup;
import com.rowanmcalpin.nextftc.core.command.groups.ParallelGroup;
import com.rowanmcalpin.nextftc.core.command.groups.ParallelRaceGroup;
import com.rowanmcalpin.nextftc.core.command.groups.SequentialGroup;
import com.rowanmcalpin.nextftc.core.command.utility.NullCommand;
import com.rowanmcalpin.nextftc.core.command.utility.delays.Delay;
import com.rowanmcalpin.nextftc.ftc.NextFTCOpMode;

import org.firstinspires.ftc.teamcode.auto.subsystems.ElevatorAngleNext;
import org.firstinspires.ftc.teamcode.auto.subsystems.HoldPosition;
import org.firstinspires.ftc.teamcode.auto.subsystems.intake.nextIntakeAngle;
import org.firstinspires.ftc.teamcode.auto.subsystems.intake.nextIntakeClaw;
import org.firstinspires.ftc.teamcode.auto.subsystems.nextLift;
import org.firstinspires.ftc.teamcode.auto.subsystems.defaultCommand;

@Autonomous(name = "emily")
public class testNext extends NextFTCOpMode {

    public testNext() {
        super(nextLift.INSTANCE, nextIntakeAngle.INSTANCE, ElevatorAngleNext.INSTANCE, nextIntakeClaw.INSTANCE,defaultCommand.INSTANCE);
    }

    public Command realRoutine(){
        return new SequentialGroup(
                ElevatorAngleNext.INSTANCE.toAngle(700, 10),
                new Delay(5),
                ElevatorAngleNext.INSTANCE.toAngle(1400,10)
        );
    }

    public Command firstRoutine()  {
        return new ParallelDeadlineGroup(
                ElevatorAngleNext.INSTANCE.toAngle(700, 10),
                defaultCommand.INSTANCE.defaultCommand(ElevatorAngleNext.INSTANCE)
        );
    }

    @Override
    public void onStartButtonPressed() {

        firstRoutine().invoke();
    }
}