package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * Created by zandr on 9/25/2017.
 */
@TeleOp(name = "TeleOp", group = "Drive")
public class TeleOp_Current extends OpMode {

    DcMotor leftwheel;
    DcMotor rightwheel;
    DcMotor liftmotor;
    Servo leftgrabber;
    Servo rightgrabber;
    double leftwheelpower;
    double rightwheelpower;
    double speedmodifier;
    double liftpower;
    double leftgrabberclosed = .65;
    double rightgrabberclosed = .35;
    double leftgrabberopen = .5;
    double rightgrabberopen = .5;
    double leftgrabberinit = .9;
    double rightgrabberinit = .1;
    double RightStick_x;
    double LeftStick_y;
    double LeftStick_x;
    double ReductionFactor;
    double dpad_speed = .16;
    double dpad_turn_speed = .22;
    double liftencoderstartpos;
    double liftencoderpos;
    double liftheight;
    double liftclearance = 1;
    double blockheight = 6;
    double liftpos;
    double ticksperrev = 560;
    double inchesperrev = 5.375;
    double ticksperinch = ticksperrev / inchesperrev;;
    int liftlevel = 0;
    boolean slowmode = false;
    boolean joystick_driving = true;


    ElapsedTime righttriggertimer = new ElapsedTime();
    ElapsedTime lefttriggertimer = new ElapsedTime();

    @Override
    public void init() {
        leftwheel = hardwareMap.dcMotor.get("left");
        rightwheel = hardwareMap.dcMotor.get("right");
        liftmotor = hardwareMap.dcMotor.get("lm");
        leftgrabber = hardwareMap.servo.get("lg");
        rightgrabber = hardwareMap.servo.get("rg");

        leftgrabber.setPosition(leftgrabberinit);
        rightgrabber.setPosition(rightgrabberinit);

        liftmotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        liftmotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        liftencoderstartpos = liftmotor.getCurrentPosition();
    }

    @Override
    public void start() {
        leftgrabber.setPosition(leftgrabberopen);
        rightgrabber.setPosition(rightgrabberopen);
    }

    @Override
    public void loop() {
        liftencoderpos = liftmotor.getCurrentPosition();

        if (gamepad1.right_trigger == 1 && righttriggertimer.seconds() > 0.5 && liftlevel < 4){
            righttriggertimer.reset();
            liftlevel += 1;
        }
        if (gamepad1.left_trigger == 1 && lefttriggertimer.seconds() > 0.5 && liftlevel > 0) {
            lefttriggertimer.reset();
            liftlevel -= 1;
        }
        if (liftlevel == 0){
            liftheight = 0;
        } else {
            liftheight = liftclearance + (liftlevel * blockheight);
        }
        liftpos = liftheight*ticksperinch;

        if(liftpos != liftmotor.getCurrentPosition()) {
            liftmotor.setTargetPosition((int) (liftpos));
            liftmotor.setPower(1);
        }

        if (gamepad1.right_bumper) {
            slowmode = true;
        }

        if (gamepad1.left_bumper) {
            slowmode = false;
        }

        if (slowmode) {
            speedmodifier = 0.25;
        } else {
            speedmodifier = 1;
        }

        if (liftencoderpos <= liftencoderstartpos){
            liftmotor.setPower(0);
        } else {
            liftpower = gamepad2.left_stick_y;
            liftmotor.setPower(liftpower * .8);
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

        if (gamepad1.dpad_up) {
            leftwheelpower = -dpad_speed;
            rightwheelpower = -dpad_speed;
        } else if (gamepad1.dpad_down) {
            leftwheelpower = dpad_speed;
            rightwheelpower = dpad_speed;
        } else if (gamepad1.dpad_left) {
            leftwheelpower = dpad_turn_speed;
            rightwheelpower = -dpad_turn_speed;
        } else if (gamepad1.dpad_right) {
            leftwheelpower = -dpad_turn_speed;
            rightwheelpower = dpad_turn_speed;
        }

        if (gamepad1.start) {
            joystick_driving = !joystick_driving;
        }

        if (joystick_driving) {
            LeftStick_y = gamepad1.left_stick_y * speedmodifier;
            LeftStick_x = gamepad1.left_stick_x * speedmodifier;
            RightStick_x = gamepad1.right_stick_x * speedmodifier;
            if (Math.abs(LeftStick_y) < .2 && Math.abs(LeftStick_x) > .2) {  //  Now we're in spinning mode
                leftwheelpower = -LeftStick_x;
                rightwheelpower = LeftStick_x;
            } else {
                if (LeftStick_y > 0) {
                    RightStick_x = -RightStick_x;
                }
                leftwheelpower = LeftStick_y - (RightStick_x / 2.02);
                rightwheelpower = LeftStick_y + (RightStick_x / 2.02);
                ReductionFactor = Math.max(Math.abs(leftwheelpower), Math.abs(rightwheelpower)) - 1;
                if (ReductionFactor > 0) {
                    if (LeftStick_y < 0) {
                        ReductionFactor = -ReductionFactor;
                    }
                    leftwheelpower = leftwheelpower - ReductionFactor;
                    rightwheelpower = rightwheelpower - ReductionFactor;
                }
            }
        } else {
            leftwheelpower = gamepad1.left_stick_y * speedmodifier;
            rightwheelpower = gamepad1.right_stick_y * speedmodifier;
        }

        leftwheel.setPower(leftwheelpower);
        rightwheel.setPower(-rightwheelpower);
    }

    @Override
    public void stop() {
        leftgrabber.setPosition(leftgrabberinit);
        rightgrabber.setPosition(rightgrabberinit);
    }
}
