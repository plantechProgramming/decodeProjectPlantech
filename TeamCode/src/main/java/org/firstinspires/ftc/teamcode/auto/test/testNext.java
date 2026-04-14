package org.firstinspires.ftc.teamcode.auto.test;

import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.commands.Commands;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.pedropathing.ivy.Scheduler;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import static com.pedropathing.ivy.Scheduler.schedule;
import static com.pedropathing.ivy.commands.Commands.*;
import static com.pedropathing.ivy.groups.Groups.*;

import org.firstinspires.ftc.teamcode.auto.subsystems.NextIntake;

//@Autonomous(name="testNext", group="test")
@TeleOp
public class testNext extends LinearOpMode {
    DcMotorEx intake;
    @Override
    public void runOpMode() {
        intake = hardwareMap.get(DcMotorEx.class, "Intake");
        NextIntake intake = new NextIntake(hardwareMap);
        intake.stop();
        //Since the scheduler is static, we need to reset it before each OpMode
        //so commands don't carry over from one OpMode to the next
//        Scheduler.reset();
        // Initialize hardware
//        Servo claw = hardwareMap.get(Servo.class, "claw");
        // Define commands
//        Command raiseArm = Command.build()
//                .setExecute(() -> armMotor.setPower(0.5))
//                .setDone(() -> armMotor.getCurrentPosition() > 1000)
//                .setEnd(endCondition -> armMotor.setPower(0))
//                .requiring(armMotor);
        // Compose: raise the arm, wait 200ms, then open the claw
//        Command sequence = sequential(
//            pickUp
//        );
//        schedule(Commands.instant(() -> intake.setPower(1.0)));
        telemetry.addLine("aaaaaa");
        telemetry.update();
        waitForStart();
        // Schedule the sequence when the OpMode starts
        while (opModeIsActive()) {
            // Run the scheduler each loop
            Scheduler.execute();
        }
    }
}