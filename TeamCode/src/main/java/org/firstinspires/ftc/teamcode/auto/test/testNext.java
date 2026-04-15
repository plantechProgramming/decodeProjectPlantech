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
@com.qualcomm.robotcore.eventloop.opmode.TeleOp
public class testNext extends LinearOpMode {
    DcMotorEx intake;
    @Override
    public void runOpMode() {
//        Scheduler.reset();

        DcMotorEx intakeMotor = hardwareMap.get(DcMotorEx.class,"Intake");
        intakeMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        intakeMotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        intakeMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        NextIntake intake = new NextIntake(hardwareMap);

        waitForStart();
        Command test = Commands.instant(()->intakeMotor.setPower(0.5));
//        Scheduler.schedule(test);
        // Schedule the sequence when the OpMode starts
        while (opModeIsActive()) {
            // Run the scheduler each loop
//            Scheduler.execute();
            telemetry.addLine("aaaaaa");
            telemetry.update();
        }
    }
}