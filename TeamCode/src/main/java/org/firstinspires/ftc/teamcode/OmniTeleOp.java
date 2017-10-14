package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * Created by Drew on 10/8/2017.
 */
@TeleOp(name = "OmniTeleOp", group = "Drive")
public class OmniTeleOp extends OpMode {

    DcMotor uwheel;
    DcMotor lwheel;
    DcMotor rwheel;
    DcMotor bwheel;
    DcMotor liftmotor;
    Servo leftgrabber;
    Servo rightgrabber;

    double leftstickx;
    double leftsticky;
    double rightstickx;
    double rightsticky;
    double liftpower;
    double leftgrabberclosed = .65;
    double rightgrabberclosed = .35;
    double leftgrabberopen = .5;
    double rightgrabberopen = .5;
    double leftgrabberinit = .9;
    double rightgrabberinit = .1;
    double speedmodifier = 0.75;

    @Override
    public void init() {
        uwheel = hardwareMap.dcMotor.get("uw");
        lwheel = hardwareMap.dcMotor.get("lw");
        rwheel = hardwareMap.dcMotor.get("rw");
        bwheel = hardwareMap.dcMotor.get("bw");
        liftmotor = hardwareMap.dcMotor.get("lm");
        leftgrabber = hardwareMap.servo.get("lg");
        rightgrabber = hardwareMap.servo.get("rg");

        leftgrabber.setPosition(leftgrabberinit);
        rightgrabber.setPosition(rightgrabberinit);

        uwheel.setDirection(DcMotor.Direction.REVERSE);
        rwheel.setDirection(DcMotor.Direction.REVERSE);

        liftmotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        uwheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lwheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rwheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        bwheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    @Override
    public void start() {
        leftgrabber.setPosition(leftgrabberopen);
        rightgrabber.setPosition(rightgrabberopen);
    }

    @Override
    public void loop() {

        leftstickx = gamepad1.left_stick_x;
        leftsticky = gamepad1.left_stick_y;
        rightstickx = gamepad1.right_stick_x;
        rightsticky = gamepad1.right_stick_y;

        liftpower = gamepad2.left_stick_y;
        liftmotor.setPower(liftpower * .8);

        if (Math.abs(leftsticky) < 0.2 && Math.abs(leftstickx) > 0.2){
            leftsticky = 0;
        }
        if (Math.abs(leftstickx) < 0.2 && Math.abs(leftsticky) > 0.2) {
            leftstickx = 0;
        }

        if (Math.abs(rightstickx) > .1) {
            rwheel.setPower(rightstickx * speedmodifier);
            bwheel.setPower(-rightstickx * speedmodifier);
            uwheel.setPower(rightstickx * speedmodifier);
            lwheel.setPower(-rightstickx * speedmodifier);
        } else {
            rwheel.setPower(leftsticky * speedmodifier);
            lwheel.setPower(leftsticky * speedmodifier);
            uwheel.setPower(leftstickx * speedmodifier);
            bwheel.setPower(leftstickx * speedmodifier);
        }

        if (gamepad2.right_bumper) {
            leftgrabber.setPosition(leftgrabberclosed);
            rightgrabber.setPosition(rightgrabberclosed);
        }
        if (gamepad2.left_bumper) {
            leftgrabber.setPosition(leftgrabberopen);
            rightgrabber.setPosition(rightgrabberopen);
        }
        if (gamepad2.y) {
            leftgrabber.setPosition(leftgrabberinit);
            rightgrabber.setPosition(rightgrabberinit);
        }

     /*   if (gamepad1.b){
            rwheel.setPower(1);
            bwheel.setPower(-1);
            uwheel.setPower(1);
            lwheel.setPower(-1);
        } else if (gamepad1.y){
            rwheel.setPower(0);
            bwheel.setPower(0);
            uwheel.setPower(0);
            lwheel.setPower(0);
        }

        if (gamepad1.x){
            rwheel.setPower(-1);
            bwheel.setPower(1);
            uwheel.setPower(-1);
            lwheel.setPower(1);
        }else if (gamepad1.y){
            rwheel.setPower(0);
            bwheel.setPower(0);
            uwheel.setPower(0);
            lwheel.setPower(0);
        }
        */

    }
}
