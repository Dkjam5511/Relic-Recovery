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
    Servo leftjewelservo;
    Servo leftjewelservoflipper;
    Servo rightjewelservo;
    Servo rightjewelservoflipper;
    Servo relicjawangle;
    Servo relicjaw;
    Servo leftclamp;
    Servo rightclamp;
    Servo leftglyphwheel;
    Servo rightglyphwheel;
    //Servo glyphspinner;

    double liftencoderstartpos;
    double relicjawanglepos = 0;
    double relicmotorpower;
    double liftencoderpos;
    double liftheight;
    double liftclearance = 3;
    double blockheight = 6;
    double ticksperrev = 560;
    double inchesperrev = 5.375;
    double ticksperinchlift = ticksperrev / inchesperrev;
    double lifttargetpos;
    double slowlifttargetpos;
    double speedmodifier = 1;

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
    boolean glyphspinnerisleft = false;
    boolean glyphintake = true;
    boolean relicanglemanualmode = false;

    ElapsedTime relicjawtimer = new ElapsedTime();
    ElapsedTime relicjawangletimer = new ElapsedTime();
    ElapsedTime manualliftmodetimer = new ElapsedTime();
    ElapsedTime relicanglemanualmodetimer = new ElapsedTime();
    ElapsedTime raiseliftimer = new ElapsedTime();
    ElapsedTime lowerliftimer = new ElapsedTime();
    ElapsedTime glyphintaketimer = new ElapsedTime();
    ElapsedTime glyphspinnertimer = new ElapsedTime();

    @Override
    public void init() {
        leftFront = hardwareMap.dcMotor.get("lf");
        rightFront = hardwareMap.dcMotor.get("rf");
        leftRear = hardwareMap.dcMotor.get("lr");
        rightRear = hardwareMap.dcMotor.get("rr");
        liftmotor = hardwareMap.dcMotor.get("lm");
        relicmotor = hardwareMap.dcMotor.get("rm");
        relicjaw = hardwareMap.servo.get("rj");
        relicjawangle = hardwareMap.servo.get("rja");
        leftclamp = hardwareMap.servo.get("lc");
        rightclamp = hardwareMap.servo.get("rc");
        leftjewelservo = hardwareMap.servo.get("ljs");
        leftjewelservoflipper = hardwareMap.servo.get("ljsf");
        rightjewelservo = hardwareMap.servo.get("rjs");
        rightjewelservoflipper = hardwareMap.servo.get("rjsf");
        leftglyphwheel = hardwareMap.servo.get("lgw");
        rightglyphwheel = hardwareMap.servo.get("rgw");
        //glyphspinner = hardwareMap.servo.get("gs");

        rightFront.setDirection(DcMotor.Direction.REVERSE);
        rightRear.setDirection(DcMotor.Direction.REVERSE);

        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        liftmotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        liftmotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        relicmotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        rightjewelservo.setPosition(GlobalVarriables.rightjewelservoinit);
        leftjewelservo.setPosition(GlobalVarriables.leftjewelservoinit);
        rightjewelservoflipper.setPosition(GlobalVarriables.rightjewelservoflipperinit);
        leftjewelservoflipper.setPosition(GlobalVarriables.leftjewelservoflipperinit);

        relicjaw.setPosition(0);
        relicjawangle.setPosition(0);

        liftencoderstartpos = liftmotor.getCurrentPosition();
    }

    @Override
    public void loop() {
        //Glyph Wheels
        if (gamepad1.left_trigger >= .75 && glyphintaketimer.seconds() > .25) {
            glyphintake = false;
            glyphintaketimer.reset();
        }

        if (gamepad1.right_trigger >= .75 && glyphintaketimer.seconds() > .25) {
            glyphintake = true;
            glyphintaketimer.reset();
        }

        if (leftclampsetting == clampclosed) {
            leftglyphwheel.setPosition(.5);
        } else {
            if (liftlevel == 0) {
                if (glyphintake) {
                    leftglyphwheel.setPosition(1);
                } else {
                    leftglyphwheel.setPosition(0);
                }
            } else {
                leftglyphwheel.setPosition(.5);
            }
        }

        if (rightclampsetting == clampclosed) {
            rightglyphwheel.setPosition(.5);
        } else {
            if (liftlevel == 0) {
                if (glyphintake) {
                    rightglyphwheel.setPosition(0);
                } else {
                    rightglyphwheel.setPosition(1);
                }
            } else {
                rightglyphwheel.setPosition(.5);
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

        telemetry.addData("r (radius or power)", wheelpower);
        telemetry.addData("stickangleradians minus 45", stickangleradians);
        telemetry.addData("radians", stickangleradians + Math.PI / 4);
        telemetry.addData("degrees", (stickangleradians + Math.PI / 4) * 180 / Math.PI);

        //Glyph Spinner
        /*
        if (gamepad1.right_trigger >= .75) {
            if (glyphspinnertimer.seconds() >= 1.50) {
                glyphspinnerisleft = !glyphspinnerisleft;
                glyphspinnertimer.reset();
                if (glyphspinnerisleft) {
                    glyphspinner.setPosition(.8);
                } else {
                    glyphspinner.setPosition(.25);
                }
            }
        } else {
            glyphspinner.setPosition(.5);
        }
*/

        //Relic Arm
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

        if (gamepad2.start && relicanglemanualmodetimer.seconds() > .25){
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
            }
            if (gamepad2.y) {
                relicjawangle.setPosition(.45);
            }
            if (gamepad2.x) {
                relicjawangle.setPosition(.4);
            }
            if (gamepad2.a) {
                relicjawangle.setPosition(.35);
            }
            if (gamepad1.y) {
                relicjawangle.setPosition(0);
            }
        }
        // Lift
        /*
        if (gamepad2.dpad_up) {
            liftlevel = 2;
            rightclampsetting = clampopen;
        }
        */
        if (gamepad2.dpad_right) {
            liftlevel = 0;
            rightclampsetting = clampopen;
        }
/*
        if (gamepad2.dpad_left) {
            liftlevel = 2;
            leftclampsetting = clampopen;
        }
        */
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
            if (gamepad2.right_stick_y <= -.9 && lowerliftimer.seconds() > .25 && liftlevel < 5) {
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
        //Glyph Grabbers
        if (gamepad2.left_trigger > .75 && leftclampsetting != clampclosed) {
            leftclampsetting = clampclosed;
            waitingforlefttriggerrelease = true;
        }
        if (waitingforlefttriggerrelease && gamepad2.left_trigger < .1) {
            if (liftlevel < 5) {
                liftlevel += 1;
                slowlifttargetpos = calculatelifttargetpos(liftlevel);
                liftslowdown = true;
            }
            waitingforlefttriggerrelease = false;

        }

        if (gamepad2.right_trigger > .75 && rightclampsetting != clampclosed) {
            rightclampsetting = clampclosed;
            waitingforrighttriggerrelease = true;
        }
        if (waitingforrighttriggerrelease && gamepad2.right_trigger < .1) {
            if (liftlevel < 5) {
                liftlevel += 1;
                slowlifttargetpos = calculatelifttargetpos(liftlevel);
                liftslowdown = true;
            }

            waitingforrighttriggerrelease = false;
        }

        if (gamepad2.left_bumper && leftclampsetting != clampinit) {
            leftclampsetting = clampinit;
        }

        if (gamepad2.right_bumper && rightclampsetting != clampinit) {
            rightclampsetting = clampinit;
        }

        if (leftclampsetting == clampinit) {
            leftclamp.setPosition(GlobalVarriables.leftclampinitpos);
        } else if (leftclampsetting == clampopen) {
            leftclamp.setPosition(GlobalVarriables.leftclampopenpos);
        } else if (leftclampsetting == clampclosed) {
            leftclamp.setPosition(GlobalVarriables.leftclampclosedpos);
        }

        if (rightclampsetting == clampinit) {
            rightclamp.setPosition(GlobalVarriables.rightclampinitpos);
        } else if (rightclampsetting == clampopen) {
            rightclamp.setPosition(GlobalVarriables.rightclampopenpos);
        } else if (rightclampsetting == clampclosed) {
            rightclamp.setPosition(GlobalVarriables.rightclampclosedpos);
        }
    }

    int calculatelifttargetpos(int liftlevel) {
        if (liftlevel == 0) {
            liftheight = 0;
        } else if (liftlevel == 1) {
            liftheight = liftclearance;
        } else if (liftlevel == 2) {
            liftheight = blockheight;
        } else {
            liftheight = liftclearance + ((liftlevel - 2) * blockheight);
        }
        return (int) (liftheight * ticksperinchlift);
    }

}
