package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * Created by Drew on 8/10/2017.
 */
@TeleOp(name="REV_Test", group="Drive")
public class REV_Test extends OpMode {

    DcMotor leftwheel;
    DcMotor rightwheel;
    double leftwheelpower;
    double rightwheelpower;


    @Override
    public void init() {
        leftwheel = hardwareMap.dcMotor.get("left");
        rightwheel = hardwareMap.dcMotor.get("right");
    }

    @Override
    public void loop() {

        leftwheelpower = gamepad1.left_stick_y;
        rightwheelpower = gamepad1.right_stick_y;

        leftwheel.setPower(leftwheelpower);
        rightwheel.setPower(-rightwheelpower);

    }


}
