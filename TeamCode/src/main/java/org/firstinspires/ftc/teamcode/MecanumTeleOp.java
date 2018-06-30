package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;


/**
 * Created by Drew on 1/9/2018.
 */
@TeleOp(name = "Mecanum TeleOp", group = "Drive")
public class MecanumTeleOp extends OpMode {
    DcMotor leftFront;
    DcMotor rightFront;
    DcMotor leftRear;
    DcMotor rightRear;
    DcMotor liftmotor;
    DcMotor relicmotor;
    DcMotor leftintake;
    DcMotor rightintake;
    Servo leftjewelservo;
    Servo leftjewelservoflipper;
    Servo rightjewelservo;
    Servo rightjewelservoflipper;
    Servo relicjawangle;
    Servo relicjaw;
    Servo leftclamp;
    Servo rightclamp;
    Servo intakeposservo;

    double liftencoderstartpos;
    double relicjawanglepos = 0;
    double relicmotorpower;
    double liftencoderpos;
    double liftheight;
    double firstliftheight = 3.5;
    double liftclearance = 4;
    double blockheight = 6;
    double ticksperrev = 560;
    double inchesperrev = 5.375;
    double ticksperinchlift = ticksperrev / inchesperrev;
    double lifttargetpos;
    double slowlifttargetpos;
    double speedmodifier = 1;
    double relicjawpos;
    double intakeposservo_closed = .94;
    double intakeposservo_open = 0;

    int liftlevel = 0;
    int rightclampsetting = 2; // 0 is closed 1 is open and 2 is init
    int leftclampsetting = 2; // 0 is closed 1 is open and 2 is init

    final int clampopen = 1;
    final int clampclosed = 0;
    final int clampinit = 2;
    final int liftvariance = 40;

    boolean relicjawshut = true;
    boolean manualliftmode = false;
    boolean liftslowdown = false;
    boolean waitingforlefttriggerrelease;
    boolean waitingforrighttriggerrelease;
    boolean glyphintake = true;
    boolean relicanglemanualmode = false;
    boolean liftup = false;
    boolean parkingintakewheels = false;

    ElapsedTime relicjawtimer = new ElapsedTime();
    ElapsedTime relicjawangletimer = new ElapsedTime();
    ElapsedTime manualliftmodetimer = new ElapsedTime();
    ElapsedTime relicanglemanualmodetimer = new ElapsedTime();
    ElapsedTime raiseliftimer = new ElapsedTime();
    ElapsedTime lowerliftimer = new ElapsedTime();
    ElapsedTime glyphintaketimer = new ElapsedTime();
    ElapsedTime wheelpowertimer = new ElapsedTime();

    @Override
    public void init() {
        leftFront = hardwareMap.dcMotor.get("lf");
        rightFront = hardwareMap.dcMotor.get("rf");
        leftRear = hardwareMap.dcMotor.get("lr");
        rightRear = hardwareMap.dcMotor.get("rr");
        liftmotor = hardwareMap.dcMotor.get("lm");
        relicmotor = hardwareMap.dcMotor.get("rm");
        leftintake = hardwareMap.dcMotor.get("li");
        rightintake = hardwareMap.dcMotor.get("ri");
        relicjaw = hardwareMap.servo.get("rj");
        relicjawangle = hardwareMap.servo.get("rja");
        leftclamp = hardwareMap.servo.get("lc");
        rightclamp = hardwareMap.servo.get("rc");
        leftjewelservo = hardwareMap.servo.get("ljs");
        leftjewelservoflipper = hardwareMap.servo.get("ljsf");
        rightjewelservo = hardwareMap.servo.get("rjs");
        rightjewelservoflipper = hardwareMap.servo.get("rjsf");
        intakeposservo = hardwareMap.servo.get("ips");

        rightFront.setDirection(DcMotor.Direction.REVERSE);
        rightRear.setDirection(DcMotor.Direction.REVERSE);

        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        liftmotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        liftmotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        relicmotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        leftintake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightintake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        rightjewelservo.setPosition(GlobalVariables.rightjewelservoinit);
        leftjewelservo.setPosition(GlobalVariables.leftjewelservoinit);
        rightjewelservoflipper.setPosition(GlobalVariables.rightjewelservoflipperinit);
        leftjewelservoflipper.setPosition(GlobalVariables.leftjewelservoflipperinit);
        leftclamp.setPosition(GlobalVariables.leftclampopenpos);
        rightclamp.setPosition(GlobalVariables.rightclampopenpos);

        relicjaw.setPosition(0);
        relicjawangle.setPosition(0);
        intakeposservo.setPosition(intakeposservo_closed);

        liftencoderstartpos = liftmotor.getCurrentPosition();
    }

    @Override
    public void loop() {
        //Glyph Wheels
        if (gamepad1.right_trigger >= .75 || parkingintakewheels) {
            glyphintake = false;
        } else {
            glyphintake = true;
        }

        if (gamepad1.left_trigger >= .75 && glyphintaketimer.seconds() > .25) {
            parkingintakewheels = !parkingintakewheels;
            glyphintaketimer.reset();
        }

        if (parkingintakewheels == false) {
            wheelpowertimer.reset();
            intakeposservo.setPosition(intakeposservo_closed);
            if (leftclampsetting != clampopen) {
                leftintake.setPower(0);
            } else {
                if (liftlevel < 3) {
                    if (glyphintake) {
                        leftintake.setPower(-1);
                    } else {
                        leftintake.setPower(1);
                    }
                } else {
                    leftintake.setPower(0);
                }
            }

            if (rightclampsetting != clampopen) {
                rightintake.setPower(0);
            } else {
                if (liftlevel < 3) {
                    if (glyphintake) {
                        rightintake.setPower(1);
                    } else {
                        rightintake.setPower(-1);
                    }
                } else {
                    rightintake.setPower(0);
                }
            }
        } else {
            intakeposservo.setPosition(intakeposservo_open);
            if (wheelpowertimer.seconds() < 1.5) {
                rightintake.setPower(-1);
                leftintake.setPower(1);
            } else {
                rightintake.setPower(0);
                leftintake.setPower(0);
            }
        }


        //Driving
        double leftstickx = 0;
        double leftsticky = 0;
        double rightstickx = 0;
        double wheelpower;
        double stickangleradians;
        double rightX;
        double leftfrontpower;
        double rightfrontpower;
        double leftrearpower;
        double rightrearpower;
        double dpadpower = .2;
        double dpadturningpower = .4;

        if (gamepad1.right_bumper) {
            speedmodifier = .5;
        }
        if (gamepad1.left_bumper) {
            speedmodifier = 1;
        }

        if (gamepad1.dpad_up) {
            leftsticky = dpadpower;
        } else if (gamepad1.dpad_right) {
            leftstickx = dpadturningpower;
        } else if (gamepad1.dpad_down) {
            leftsticky = -dpadpower;
        } else if (gamepad1.dpad_left) {
            leftstickx = -dpadturningpower;
        } else {
            leftstickx = gamepad1.left_stick_x * speedmodifier;
            leftsticky = -gamepad1.left_stick_y * speedmodifier;
            rightstickx = gamepad1.right_stick_x * speedmodifier;
        }
        if (Math.abs(leftsticky) <= .15) {
            leftsticky = 0;
        }
        wheelpower = Math.hypot(leftstickx, leftsticky);
        stickangleradians = Math.atan2(leftsticky, leftstickx);

        stickangleradians = stickangleradians - Math.PI / 4; //adjust by 45 degrees

        rightX = rightstickx * .5;
        leftfrontpower = wheelpower * Math.cos(stickangleradians) + rightX;
        rightfrontpower = wheelpower * Math.sin(stickangleradians) - rightX;
        leftrearpower = wheelpower * Math.sin(stickangleradians) + rightX;
        rightrearpower = wheelpower * Math.cos(stickangleradians) - rightX;

        leftFront.setPower(leftfrontpower);
        rightFront.setPower(rightfrontpower);
        leftRear.setPower(leftrearpower);
        rightRear.setPower(rightrearpower);

        //Relic Arm
        relicmotorpower = gamepad2.left_stick_y;
        relicmotor.setPower(relicmotorpower);

        if (relicjawtimer.seconds() > .4 && gamepad2.b) {
            relicjawshut = !relicjawshut;
            relicjawtimer.reset();
        }

        if (!relicjawshut) {
            relicjaw.setPosition(relicjawpos);
        } else {
            relicjaw.setPosition(0);
        }

        if (gamepad2.back && relicanglemanualmodetimer.seconds() > .25) {
            relicanglemanualmode = !relicanglemanualmode;
            relicanglemanualmodetimer.reset();
        }
        if (relicanglemanualmode) {
            if (gamepad2.y && relicjawangletimer.seconds() > .25 && relicjawanglepos < 1) {
                relicjawanglepos = relicjawanglepos + .05;
                relicjawangletimer.reset();
            } else if (gamepad2.a && relicjawangletimer.seconds() > .25 && relicjawanglepos > 0) {
                relicjawanglepos = relicjawanglepos - .05;
                relicjawangletimer.reset();
            }
            relicjawangle.setPosition(relicjawanglepos);
        } else {
            if (gamepad2.dpad_up) {
                relicjawangle.setPosition(1);
                relicjawanglepos = 1;
                relicjawpos = .5;
            }
            if (gamepad2.y) {
                relicjawangle.setPosition(.45);
                relicjawanglepos = .45;
                relicjawpos = .35;
            }
            if (gamepad2.x) {
                relicjawangle.setPosition(.40);
                relicjawanglepos = .40;
                relicjawpos = .45;
            }
            if (gamepad2.a) {
                relicjawangle.setPosition(.30);
                relicjawanglepos = .30;
                relicjawpos = .7;
            }
            if (gamepad1.y) {
                relicjawangle.setPosition(0);
                relicjawanglepos = 0;
                relicjawpos = .7;
            }
        }
        // Lift
        if (gamepad2.dpad_right) {
            liftlevel = 0;
            rightclampsetting = clampopen;
        }

        if (gamepad2.dpad_down) {
            liftlevel = 0;
            leftclampsetting = clampopen;
        }

        if (gamepad1.back && manualliftmodetimer.seconds() > .4) {
            manualliftmode = !manualliftmode;
            manualliftmodetimer.reset();
        }
        if (gamepad1.x) {
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
            if (gamepad2.right_stick_y <= -.9 && lowerliftimer.seconds() > .25 && liftlevel < 6) {
                lowerliftimer.reset();
                liftlevel += 1;
            }

            lifttargetpos = calculatelifttargetpos(liftlevel);

            if (liftencoderpos > slowlifttargetpos - liftvariance) {
                liftslowdown = false;
            }

            if (lifttargetpos < liftencoderpos - liftvariance) {
                if (liftslowdown) {
                    liftmotor.setPower(-.3);
                } else if (liftencoderpos - lifttargetpos > 120) {
                    liftmotor.setPower(-.8);
                } else {
                    liftmotor.setPower(-.3);
                }

            } else if (lifttargetpos > liftencoderpos + liftvariance) {
                if (liftslowdown) {
                    liftmotor.setPower(.6);
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
        //Glyph Grabbers
        if (gamepad2.left_trigger > .75 && leftclampsetting != clampclosed) {
            if (liftlevel == 1) {
                liftlevel = 0;
            }
            leftclampsetting = clampclosed;
            waitingforlefttriggerrelease = true;
        }
        if (waitingforlefttriggerrelease && gamepad2.left_trigger < .1) {
            if (liftlevel < 6) {
                liftup = true;
                slowlifttargetpos = calculatelifttargetpos(liftlevel);
                liftslowdown = true;
            }
            waitingforlefttriggerrelease = false;

        }

        if (gamepad2.right_trigger > .75 && rightclampsetting != clampclosed) {
            if (liftlevel == 1) {
                liftlevel = 0;
            }
            rightclampsetting = clampclosed;
            waitingforrighttriggerrelease = true;
        }
        if (waitingforrighttriggerrelease && gamepad2.right_trigger < .1) {
            if (liftlevel < 6) {
                liftup = true;
                slowlifttargetpos = calculatelifttargetpos(liftlevel);
                liftslowdown = true;
            }
            waitingforrighttriggerrelease = false;
        }

        if (liftup) {
            if (liftlevel == 0) {
                liftlevel = 1;
            }
            liftup = false;
        }

        if (gamepad2.left_bumper && leftclampsetting != clampinit) {
            leftclampsetting = clampinit;
        }

        if (gamepad2.right_bumper && rightclampsetting != clampinit) {
            rightclampsetting = clampinit;
        }

        if (leftclampsetting == clampinit) {
            leftclamp.setPosition(GlobalVariables.leftclampinitpos);
        } else if (leftclampsetting == clampopen) {
            leftclamp.setPosition(GlobalVariables.leftclampopenpos);
        } else if (leftclampsetting == clampclosed) {
            leftclamp.setPosition(GlobalVariables.leftclampclosedpos);
        }

        if (rightclampsetting == clampinit) {
            rightclamp.setPosition(GlobalVariables.rightclampinitpos);
        } else if (rightclampsetting == clampopen) {
            rightclamp.setPosition(GlobalVariables.rightclampopenpos);
        } else if (rightclampsetting == clampclosed) {
            rightclamp.setPosition(GlobalVariables.rightclampclosedpos);
        }
    }

    int calculatelifttargetpos(int liftlevel) {
        if (liftlevel == 0) {
            liftheight = 0;
        } else if (liftlevel == 1) {
            liftheight = firstliftheight;
        } else if (liftlevel == 2) {
            liftheight = blockheight + 1;
        } else {
            liftheight = liftclearance + ((liftlevel - 2) * blockheight);
        }
        return (int) (liftheight * ticksperinchlift);
    }

}
