package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import static org.firstinspires.ftc.teamcode.GlobalVarriables.leftgrabberopen;

/**
 * Created by zandr on 9/25/2017.
 */
@TeleOp(name = "TeleOp", group = "Drive")
public class TeleOp_Current extends OpMode {

    DcMotor leftwheel;
    DcMotor rightwheel;
    DcMotor liftmotor;
    DcMotor relicmotor;
    Servo leftgrabber;
    Servo rightgrabber;
    Servo jewelservo;
    Servo relicjawangle;
    Servo relicjaw;
    ColorSensor ljewelcs;
    double leftwheelpower;
    double rightwheelpower;
    double relicmotorpower;
    double speedmodifier = 1;
    double RightStick_x;
    double LeftStick_y;
    double LeftStick_x;
    double PrevLeftStick_y = .1;
    double PrevLeftStick_x = .1;
    double ReductionFactor;
    double liftencoderstartpos;
    double liftencoderpos;
    double liftheight;
    double liftclearance = 3;
    double blockheight = 6;
    double lifttargetpos;
    double ticksperrev = 560;
    double inchesperrev = 5.375;
    double ticksperinchlift = ticksperrev / inchesperrev;
    int relicjawanglepos;
    int reversevalue = 1;
    int liftlevel = 0;
    int glyphgrabbersetting = 2;
    boolean slowmode = false;
    boolean joystick_driving = true;
    boolean leftbumperheld;
    boolean relicjawshut = true;
    boolean reverse = false;

    //Timers
    ElapsedTime rightbumpertimer = new ElapsedTime();
    ElapsedTime righttriggertimer = new ElapsedTime();
    ElapsedTime leftbumperheldtimer = new ElapsedTime();
    ElapsedTime lefttriggertimer = new ElapsedTime();
    ElapsedTime reversetimer = new ElapsedTime();
    ElapsedTime relicjawangletimer = new ElapsedTime();
    

    @Override
    public void init() {
        // Hardware Maps
        leftwheel = hardwareMap.dcMotor.get("left");
        rightwheel = hardwareMap.dcMotor.get("right");
        liftmotor = hardwareMap.dcMotor.get("lm");
        relicmotor = hardwareMap.dcMotor.get("rm");
        leftgrabber = hardwareMap.servo.get("lg");
        rightgrabber = hardwareMap.servo.get("rg");
        jewelservo = hardwareMap.servo.get("js");
        relicjaw = hardwareMap.servo.get("rj");
        relicjawangle = hardwareMap.servo.get("rga");
        ljewelcs = hardwareMap.colorSensor.get("jcs");

        ljewelcs.enableLed(false);

        leftgrabber.setPosition(GlobalVarriables.leftgrabberinit);
        rightgrabber.setPosition(GlobalVarriables.rightgrabberinit);

        jewelservo.setPosition(.6);
        
        relicjaw.setPosition(0);
        relicjawangle.setPosition(0);

        leftwheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightwheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        liftmotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        liftmotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        liftmotor.setDirection(DcMotor.Direction.REVERSE);

        relicmotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        liftencoderstartpos = liftmotor.getCurrentPosition();
    }

    @Override
    public void start() {
    }

    @Override
    public void loop() {

        // General Driving Controls
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

        if (gamepad1.a && reversetimer.seconds() > .4){
            reverse = ! reverse;
            reversetimer.reset();
        }

        if (reverse){
            reversevalue = -1;
        } else {
            reversevalue = 1;
        }

        // Relic Arm
        relicmotorpower = gamepad2.left_stick_y / 2;
        relicmotor.setPower(relicmotorpower);

        if (lefttriggertimer.seconds() > .4 && gamepad2.left_trigger >= .75){
            relicjawshut = !relicjawshut;
            lefttriggertimer.reset();
        }

        if (!relicjawshut){
            relicjaw.setPosition(.5);
        } else {
            relicjaw.setPosition(0);
        }

        if (gamepad2.y && relicjawangletimer.seconds() > .25 && relicjawanglepos < 4){
            relicjawanglepos += 1;
            relicjawangletimer.reset();
        } else if (gamepad2.a && relicjawangletimer.seconds() > .25 && relicjawanglepos > 0){
            relicjawanglepos -= 1;
            relicjawangletimer.reset();
        }

        if (relicjawanglepos == 0){
            relicjawangle.setPosition(0);
        } else if (relicjawanglepos == 1){
            relicjawangle.setPosition(.325);
        } else if (relicjawanglepos == 2){
            relicjawangle.setPosition(.4);
        } else if (relicjawanglepos == 3){
            relicjawangle.setPosition(.6);
        } else if (relicjawanglepos == 4){
            relicjawangle.setPosition(.8);
        }


        //Lift
        liftencoderpos = liftmotor.getCurrentPosition() - liftencoderstartpos;

        if (gamepad2.right_bumper && rightbumpertimer.seconds() > 0.25 && liftlevel < 4) {
            rightbumpertimer.reset();
            liftlevel += 1;
        }
        if (gamepad2.right_trigger == 1 && righttriggertimer.seconds() > 0.25 && liftlevel > 0) {
            righttriggertimer.reset();
            liftlevel -= 1;
        }
        if (liftlevel == 0) {
            liftheight = 0;
        } else {
            liftheight = liftclearance + ((liftlevel - 1) * blockheight);
        }
        lifttargetpos = liftheight * ticksperinchlift;

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

        // Glyph Grabber and Glyph Pushers
        if (gamepad2.left_bumper && !leftbumperheld) { // Checking if the left bumper is staring to be held
            leftbumperheldtimer.reset();
            leftbumperheld = true;
        } else if (!gamepad2.left_bumper && leftbumperheld) { // Checking if the left bumper has been released and changing the grabber position based on the time held
            if (leftbumperheldtimer.seconds() >= 1.5 && glyphgrabbersetting != 2) {
                glyphgrabbersetting = 2;
            } else if (glyphgrabbersetting != 1) {
                glyphgrabbersetting = 1;
            } else if (glyphgrabbersetting != 0) {
                glyphgrabbersetting = 0;
            }
            leftbumperheld = false;
        }

        if (glyphgrabbersetting == 1) {
            leftgrabber.setPosition(GlobalVarriables.leftgrabberopen);
            rightgrabber.setPosition(GlobalVarriables.rightgrabberopen);
        } else if (glyphgrabbersetting == 0) {
            leftgrabber.setPosition(GlobalVarriables.leftgrabberclosed);
            rightgrabber.setPosition(GlobalVarriables.rightgrabberclosed);
        } else if (glyphgrabbersetting == 2) {
            leftgrabber.setPosition(GlobalVarriables.leftgrabberinit);
            rightgrabber.setPosition(GlobalVarriables.rightgrabberinit);
        }


        //One Joystick Driving
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
            if (Math.abs(LeftStick_y) < .1 && Math.abs(LeftStick_x) > .1) {//  Now we're in spinning mode
                leftwheelpower = -LeftStick_x * reversevalue;
                rightwheelpower = LeftStick_x * reversevalue;
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

        leftwheel.setPower(leftwheelpower * reversevalue);
        rightwheel.setPower(-rightwheelpower * reversevalue);


        telemetry.addData("lift level", liftlevel);
        telemetry.update();
    }

    @Override
    public void stop() {
        leftgrabber.setPosition(GlobalVarriables.leftgrabberinit);
        rightgrabber.setPosition(GlobalVarriables.rightgrabberinit);
    }
}
