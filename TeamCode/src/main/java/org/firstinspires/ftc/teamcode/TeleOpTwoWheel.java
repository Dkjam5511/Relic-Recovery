package org.firstinspires.ftc.teamcode;

import android.provider.Settings;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * Created by zandr on 9/25/2017.
 */
@Disabled
public class TeleOpTwoWheel extends OpMode {

    DcMotor leftwheel;
    DcMotor rightwheel;
    DcMotor liftmotor;
    DcMotor relicmotor;
    Servo jewelservo;
    Servo sidejewelservo;
    Servo relicjawangle;
    Servo relicjaw;
    Servo leftclamp;
    Servo rightclamp;
    ColorSensor ljewelcs;
    double leftwheelpower;
    double rightwheelpower;
    double relicmotorpower;
    double speedmodifier = 1;
    double dpad_speed = .3;
    double dpad_turn_speed = .25;
    double RightStick_x;
    double LeftStick_y;
    double LeftStick_x;
    double PrevLeftStick_y = 0;
    double PrevLeftStick_x = 0;
    double PrevRightStick_x = 0;
    double HighestWheelPower;
    double liftencoderstartpos;
    double liftencoderpos;
    double liftheight;
    double liftclearance = 3;
    double blockheight = 6;
    double lifttargetpos;
    double ticksperrev = 560;
    double inchesperrev = 5.375;
    double ticksperinchlift = ticksperrev / inchesperrev;
    double relicjawanglepos = 0;
    int reversevalue = 1;
    int liftlevel = 0;
    int rightclampsetting = 2; // 0 is closed 1 is open and 2 is init
    int leftclampsetting = 2; // 0 is closed 1 is open and 2 is init
    boolean joystick_driving = true;
    boolean relicjawshut = true;
    boolean manualliftmode = false;
    boolean waitingforlefttriggerrelease;
    boolean waitingforrighttriggerrelease;
    boolean liftslowdown = false;

    //Timers
    ElapsedTime raiseliftimer = new ElapsedTime();
    ElapsedTime lowerliftimer = new ElapsedTime();
    ElapsedTime relicjawtimer = new ElapsedTime();
    ElapsedTime reversetimer = new ElapsedTime();
    ElapsedTime relicjawangletimer = new ElapsedTime();
    ElapsedTime joystickdrivingtimer = new ElapsedTime();
    ElapsedTime manualliftmodetimer = new ElapsedTime();
    ElapsedTime leftstickxtimer = new ElapsedTime();
    ElapsedTime leftstickytimer = new ElapsedTime();
    ElapsedTime rightstickxtimer = new ElapsedTime();


    double GetStick(double newstickvalue, double prevstickvalue, double sensitivityvalue, ElapsedTime timesinceaccel) {
        double returnstickvalue;
        returnstickvalue = prevstickvalue;
        if (timesinceaccel.seconds() > .001) {
            if ((Math.abs(newstickvalue - prevstickvalue) > sensitivityvalue)) {
                if (newstickvalue > prevstickvalue) {
                    returnstickvalue = prevstickvalue + sensitivityvalue;
                } else {
                    returnstickvalue = prevstickvalue - sensitivityvalue;
                }
            } else {
                returnstickvalue = newstickvalue;  
            }

        }
        return returnstickvalue;
    }


    @Override
    public void init() {
        // Hardware Maps
        leftwheel = hardwareMap.dcMotor.get("left");
        rightwheel = hardwareMap.dcMotor.get("right");
        liftmotor = hardwareMap.dcMotor.get("lm");
        relicmotor = hardwareMap.dcMotor.get("rm");
        sidejewelservo = hardwareMap.servo.get("sjs");
        jewelservo = hardwareMap.servo.get("js");
        relicjaw = hardwareMap.servo.get("rj");
        relicjawangle = hardwareMap.servo.get("rja");
        leftclamp = hardwareMap.servo.get("lc");
        rightclamp = hardwareMap.servo.get("rc");

        //Initializing Positions

        rightclamp.setPosition(GlobalVarriables.rightclampinit);
        leftclamp.setPosition(GlobalVarriables.leftclampinit);

        jewelservo.setPosition(.6);
        sidejewelservo.setPosition(.2);

        relicjaw.setPosition(0);
        relicjawangle.setPosition(0);

        leftwheel.setDirection(DcMotor.Direction.REVERSE);
        leftwheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        rightwheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        liftmotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        liftmotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        relicmotor.setDirection(DcMotor.Direction.REVERSE);
        relicmotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        liftencoderstartpos = liftmotor.getCurrentPosition();
    }

    @Override
    public void start() {

    }

    @Override
    public void loop() {

        // General Driving Controls

        if (gamepad1.right_bumper) {   // Slow Mode
            speedmodifier = 0.4;
        }

        if (gamepad1.left_bumper) {    // Not Slow Mode
            speedmodifier = 1;
        }

        if (gamepad1.a && reversetimer.seconds() > .4) {
            reversevalue = reversevalue * -1;
            reversetimer.reset();
        }

        // Relic Arm
        relicmotorpower = gamepad2.left_stick_y;
        relicmotor.setPower(relicmotorpower);

        if (relicjawtimer.seconds() > .4 && gamepad2.b) {
            relicjawshut = !relicjawshut;
            relicjawtimer.reset();
        }

        if (!relicjawshut) {
            relicjaw.setPosition(.5);
        } else {
            relicjaw.setPosition(0);
        }

        if (gamepad2.y && relicjawangletimer.seconds() > .25 && relicjawanglepos < 1) {
            relicjawanglepos = relicjawanglepos + .05;
            relicjawangletimer.reset();
        } else if (gamepad2.a && relicjawangletimer.seconds() > .25 && relicjawanglepos > 0) {
            relicjawanglepos = relicjawanglepos - .05;
            relicjawangletimer.reset();
        }

        relicjawangle.setPosition(relicjawanglepos);

        //Lift
        if (gamepad2.dpad_up) {
            liftlevel = 2;
            rightclampsetting = 1;
        }
        if (gamepad2.dpad_right) {
            liftlevel = 0;
            rightclampsetting = 1;
        }

        if (gamepad2.dpad_left) {
            liftlevel = 2;
            leftclampsetting = 1;
        }
        if (gamepad2.dpad_down) {
            liftlevel = 0;
            leftclampsetting = 1;
        }

        if (gamepad2.back && manualliftmodetimer.seconds() > .4) {
            manualliftmode = !manualliftmode;
            manualliftmodetimer.reset();
        }
        if (gamepad2.x) {
            liftencoderstartpos = liftmotor.getCurrentPosition();
        }

        if (manualliftmode) {
            liftmotor.setPower(-gamepad2.right_stick_y);
        } else {
            liftencoderpos = liftmotor.getCurrentPosition() - liftencoderstartpos;
            if (gamepad2.right_stick_y >= .9 && raiseliftimer.seconds() > .25 && liftlevel > 0) {
                raiseliftimer.reset();
                liftlevel -= 1;
            }
            if (gamepad2.right_stick_y <= -.9 && lowerliftimer.seconds() > .25 && liftlevel < 5) {
                lowerliftimer.reset();
                liftlevel += 1;
            }
            if (liftlevel == 0) {
                liftheight = 0;
            } else if (liftlevel == 1) {
                liftheight = liftclearance;
            } else if (liftlevel == 2) {
                liftheight = blockheight;
            } else {
                liftheight = liftclearance + ((liftlevel - 2) * blockheight);
            }
            lifttargetpos = liftheight * ticksperinchlift;

            if (lifttargetpos < liftencoderpos - 40) {
                if (liftslowdown) {
                    liftmotor.setPower(-.3);
                } else if (liftencoderpos - lifttargetpos > 120) {
                    liftmotor.setPower(-.8);
                } else {
                    liftmotor.setPower(-.3);
                }

            } else if (lifttargetpos > liftencoderpos + 40) {
                if (liftslowdown) {
                    liftmotor.setPower(.3);
                } else if (lifttargetpos - liftencoderpos > 120) {
                    liftmotor.setPower(1);
                } else {
                    liftmotor.setPower(.6);
                }
            } else {
                liftmotor.setPower(0);
                liftslowdown = false;
            }

        }

        // Glyph Grabbers
        if (gamepad2.left_trigger > .75 && leftclampsetting != 0) {
            leftclampsetting = 0;
            waitingforlefttriggerrelease = true;
        }
        if (waitingforlefttriggerrelease && gamepad2.left_trigger < .1) {
            if (liftlevel < 5) {
                liftlevel += 1;
            }
            waitingforlefttriggerrelease = false;
            liftslowdown = true;
        }

        if (gamepad2.right_trigger > .75 && rightclampsetting != 0) {
            rightclampsetting = 0;
            waitingforrighttriggerrelease = true;
        }
        if (waitingforrighttriggerrelease && gamepad2.right_trigger < .1) {
            if (liftlevel < 5) {
                liftlevel += 1;
            }
            liftslowdown = true;
            waitingforrighttriggerrelease = false;
        }

        if (gamepad2.left_bumper && leftclampsetting != 2) {
            leftclampsetting = 2;
        }

        if (gamepad2.right_bumper && rightclampsetting != 2) {
            rightclampsetting = 2;
        }

        if (leftclampsetting == 2) {
            leftclamp.setPosition(GlobalVarriables.leftclampinit);
        } else if (leftclampsetting == 1) {
            leftclamp.setPosition(GlobalVarriables.leftclampopen);
        } else if (leftclampsetting == 0){
            leftclamp.setPosition(GlobalVarriables.leftclampclosed);
        }

        if (rightclampsetting == 2) {
            rightclamp.setPosition(GlobalVarriables.rightclampinit);
        } else if (rightclampsetting == 1) {
            rightclamp.setPosition(GlobalVarriables.rightclampopen);
        } else if (rightclampsetting == 0){
            rightclamp.setPosition(GlobalVarriables.rightclampclosed);
        }


        //One Joystick Driving
        if (gamepad1.back && joystickdrivingtimer.seconds() > .4) {
            joystick_driving = !joystick_driving;
            joystickdrivingtimer.reset();
        }

        if (joystick_driving) {

            LeftStick_y = GetStick(gamepad1.left_stick_y * -.8, PrevLeftStick_y, .1, leftstickytimer);  // multiply by -1 to flip y so that positive is up, negative is down
            if (PrevLeftStick_y != LeftStick_y) {
                PrevLeftStick_y = LeftStick_y;
                leftstickytimer.reset();
            }

            LeftStick_x = GetStick(gamepad1.left_stick_x * .8, PrevLeftStick_x, .05, leftstickxtimer);
            if (PrevLeftStick_x != LeftStick_x) {
                PrevLeftStick_x = LeftStick_x;
                leftstickxtimer.reset();
            }

            RightStick_x = GetStick(gamepad1.right_stick_x * .8, PrevRightStick_x, .05, rightstickxtimer);
            if (PrevRightStick_x != RightStick_x) {
                PrevRightStick_x = RightStick_x;
                rightstickxtimer.reset();
            }

            if (Math.abs(LeftStick_y) < .1 && Math.abs(LeftStick_x) > .1) {  //  Now we're in spinning mode
                leftwheelpower = LeftStick_x * reversevalue;
                rightwheelpower = -LeftStick_x * reversevalue;
            } else {  // Left stick forward/backward driving mode, ignores left stick x, turns using right stick x
                leftwheelpower = LeftStick_y;
                rightwheelpower = LeftStick_y;
                if (Math.abs(RightStick_x) > 0) {
                    RightStick_x = RightStick_x * reversevalue / 1.8;
                    rightwheelpower = rightwheelpower - RightStick_x;
                    leftwheelpower = leftwheelpower + RightStick_x;
                    HighestWheelPower = Math.max(Math.abs(leftwheelpower), Math.abs(rightwheelpower));  // keep wheel power from going over 1
                    if (HighestWheelPower > 1) {
                        leftwheelpower = leftwheelpower / HighestWheelPower;
                        rightwheelpower = rightwheelpower / HighestWheelPower;
                    }
                }
            }
        } else {  // Simple 2 joystick driving
            leftwheelpower = gamepad1.left_stick_y;
            rightwheelpower = gamepad1.right_stick_y;
        }

        if (gamepad1.dpad_up) {
            leftwheelpower = dpad_speed;
            rightwheelpower = dpad_speed;
        } else if (gamepad1.dpad_down) {
            leftwheelpower = -dpad_speed;
            rightwheelpower = -dpad_speed;
        } else if (gamepad1.dpad_left) {
            leftwheelpower = -dpad_turn_speed;
            rightwheelpower = dpad_turn_speed;
        } else if (gamepad1.dpad_right) {
            leftwheelpower = dpad_turn_speed;
            rightwheelpower = -dpad_turn_speed;
        }
        leftwheel.setPower(leftwheelpower * reversevalue * speedmodifier);
        rightwheel.setPower(rightwheelpower * reversevalue * speedmodifier);


        telemetry.addData("lift level", liftlevel);
        telemetry.addData("Reverse Value", reversevalue);
        telemetry.addData("Relic Jaw Angle Pos", relicjawanglepos);

        telemetry.update();
    }

    @Override
    public void stop() {

    }
}
