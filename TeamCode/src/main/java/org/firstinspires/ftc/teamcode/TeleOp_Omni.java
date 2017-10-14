package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

/**
 * Created by zandr on 10/2/2017.
 * Some TeleOp prototype code for a possible design in the future.
 *It needs a few major fixes that could be done with testing. Not for use as of now.
 */
@TeleOp(name="TeleOp_Omnidirectional", group="Drive")
public class TeleOp_Omni extends OpMode {

    DcMotor tlwheel;
    DcMotor trwheel;
    DcMotor blwheel;
    DcMotor brwheel;
    double tlwheelpower;
    double trwheelpower;
    double blwheelpower;
    double brwheelpower;


    @Override
    public void init() {
        tlwheel = hardwareMap.dcMotor.get("tl");
        trwheel = hardwareMap.dcMotor.get("tr");
        blwheel = hardwareMap.dcMotor.get("bl");
        blwheel = hardwareMap.dcMotor.get("br");
    }

    @Override
    public void loop() {

        tlwheelpower = gamepad1.left_stick_y;
        blwheelpower = gamepad1.left_stick_x;
        trwheelpower = gamepad1.right_stick_x;
        brwheelpower = gamepad1.right_stick_y;


        tlwheel.setPower(tlwheelpower);
        trwheel.setPower(trwheelpower);
        blwheel.setPower(blwheelpower);
        brwheel.setPower(brwheelpower);

    }


}
