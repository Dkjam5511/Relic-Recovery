package org.firstinspires.ftc.teamcode.Tests;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
@TeleOp (name = "Jewel Servo Test", group = "Tests")
public class Jewel_Arm_Test extends OpMode {

    Servo leftjewelservo;
    double servopos = .5;

    ElapsedTime timer = new ElapsedTime();
    @Override
    public void init() {
        leftjewelservo = hardwareMap.servo.get("ljs");
        leftjewelservo.setPosition(.5);
    }

    @Override
    public void loop() {
        if (gamepad1.y && timer.seconds() > .2){
            servopos = servopos + .02;
            timer.reset();
        }
        if (gamepad1.a && timer.seconds() > .2){
            servopos = servopos - .02;
            timer.reset();
        }

        leftjewelservo.setPosition(servopos);

        telemetry.addData("Servo Pos", servopos);

    }
}
