package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
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
    Servo jewelservo;
    double leftwheelpower;
    double rightwheelpower;
    double speedmodifier = 1;
    double leftgrabberclosed = .75;
    double rightgrabberclosed = .25;
    double leftgrabberopen = .4;
    double rightgrabberopen = .6;
    double leftgrabberinit = .9;
    double rightgrabberinit = .1;
    double RightStick_x;
    double LeftStick_y;
    double LeftStick_x;
    double PrevLeftStick_y = .1;
    double PrevLeftStick_x = .1;
    double ReductionFactor;
    double dpad_speed = .16;
    double dpad_turn_speed = .22;
    double liftencoderstartpos;
    double liftencoderpos;
    double liftheight;
    double liftclearance = 2;
    double blockheight = 6;
    double lifttargetpos;
    double ticksperrev = 560;
    double inchesperrev = 5.375;
    double ticksperinch = ticksperrev / inchesperrev;
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
        jewelservo = hardwareMap.servo.get("js");

        leftgrabber.setPosition(leftgrabberinit);
        rightgrabber.setPosition(rightgrabberinit);

        jewelservo.setPosition(.5);

        leftwheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightwheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        liftmotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        liftmotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        liftmotor.setDirection(DcMotor.Direction.REVERSE);

        liftencoderstartpos = liftmotor.getCurrentPosition();
    }

    @Override
    public void start() {
        leftgrabber.setPosition(leftgrabberopen);
        rightgrabber.setPosition(rightgrabberopen);
    }

    @Override
    public void loop() {
        liftencoderpos = liftmotor.getCurrentPosition() - liftencoderstartpos;

        if (gamepad2.right_trigger == 1 && righttriggertimer.seconds() > 0.25 && liftlevel < 4) {
            righttriggertimer.reset();
            liftlevel += 1;
        }
        if (gamepad2.left_trigger == 1 && lefttriggertimer.seconds() > 0.25 && liftlevel > 0) {
            lefttriggertimer.reset();
            liftlevel -= 1;
        }
        if (liftlevel == 0) {
            liftheight = 0;
        } else {
            liftheight = liftclearance + ((liftlevel - 1) * blockheight);
        }
        lifttargetpos = liftheight * ticksperinch;

        if (lifttargetpos < liftencoderpos - 40) {
            if (liftencoderpos - lifttargetpos > 120) {
                liftmotor.setPower(-.8);
            } else {
                liftmotor.setPower(-.3);
            }

        } else if (lifttargetpos > liftencoderpos + 40) {
            if (lifttargetpos - liftencoderpos > 120) {
                liftmotor.setPower(1);
            } else {
                liftmotor.setPower(.6);
            }
        } else {
            liftmotor.setPower(0);
        }

        telemetry.addData("lifttargetpos", lifttargetpos);
        telemetry.addData("liftencoderpos", liftencoderpos);
        telemetry.addData("liftencoderstartpos", liftencoderstartpos);
        telemetry.addData("lift level", liftlevel);
        telemetry.addData("lift height", liftheight);
        telemetry.update();

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

        if (gamepad1.back) {
            joystick_driving = !joystick_driving;
        }

        if (joystick_driving) {

            LeftStick_y = gamepad1.left_stick_y * speedmodifier;

            if (LeftStick_y > 0 && LeftStick_y > PrevLeftStick_y) {
                LeftStick_y = PrevLeftStick_y + .01;
                if (LeftStick_y < .1) {
                    LeftStick_y = .1;
                }
            } else if (LeftStick_y < 0 && LeftStick_y < PrevLeftStick_y) {
                LeftStick_y = PrevLeftStick_y - .01;
                if (LeftStick_y > -.1) {
                    LeftStick_y = -.1;
                }
            }
            PrevLeftStick_y = LeftStick_y;

            LeftStick_x = gamepad1.left_stick_x * speedmodifier;

            if (LeftStick_x > 0 && LeftStick_x > PrevLeftStick_x) {
                LeftStick_x = PrevLeftStick_x + .01;
                if (LeftStick_x < .1) {
                    LeftStick_x = .1;
                }
            } else if (LeftStick_x < 0 && LeftStick_x < PrevLeftStick_x) {
                LeftStick_x = PrevLeftStick_x - .01;
                if (LeftStick_x > -.1) {
                    LeftStick_x = -.1;
                }
            }
            PrevLeftStick_x = LeftStick_x;

            RightStick_x = gamepad1.right_stick_x * speedmodifier;
            if (Math.abs(LeftStick_y) < .1 && Math.abs(LeftStick_x) > .1) {  //  Now we're in spinning mode
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
