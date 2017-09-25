package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * Created by zandr on 9/25/2017.
 */

public class TeleOp extends OpMode{

    DcMotor leftwheel;
    DcMotor rightwheel;
    Servo leftgrabber;
    Servo rightgrabber;
    double leftwheelpower;
    double rightwheelpower;

    @Override
    public void init() {
        leftwheel = hardwareMap.dcMotor.get("left");
        rightwheel = hardwareMap.dcMotor.get("right");
        leftgrabber = hardwareMap.servo.get("lg");
        rightgrabber = hardwareMap.servo.get("rg");

        leftgrabber.setPosition(0);
        rightgrabber.setPosition(0);
    }

    @Override
    public void loop() {
        leftwheelpower = gamepad1.left_stick_y;
        rightwheelpower = gamepad1.right_stick_y;
        leftwheel.setPower(leftwheelpower);
        rightwheel.setPower(-rightwheelpower);

        if (gamepad1.right_trigger == 1){
            leftgrabber.setPosition(90);
            rightgrabber.setPosition(-90);
        }
        if (gamepad1.left_trigger == 1){
            leftgrabber.setPosition(0);
            rightgrabber.setPosition(0);
        }
    }
}
