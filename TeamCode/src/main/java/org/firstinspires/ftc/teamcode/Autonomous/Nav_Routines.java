package org.firstinspires.ftc.teamcode.Autonomous;

import android.graphics.Bitmap;
import android.graphics.Color;
import android.os.Environment;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.vuforia.Image;
import com.vuforia.PIXEL_FORMAT;
import com.vuforia.Vuforia;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;
import org.firstinspires.ftc.teamcode.DbgLog;
import org.firstinspires.ftc.teamcode.GlobalVarriables;

import java.io.File;
import java.io.FileOutputStream;
import java.io.IOException;

/**
 * Created by Drew on 9/17/2017.
 * :D
 */

public abstract class Nav_Routines extends LinearOpMode {
    DcMotor leftWheel;
    DcMotor rightWheel;
    DcMotor liftmotor;
    DcMotor relicmotor;
    public Servo jewelservo;
    public Servo sidejewelservo;
    public Servo leftclamp;
    public Servo rightclamp;
    ColorSensor jewelcsright;
    ColorSensor jewelcsleft;
    ColorSensor sidejewelcs;
    BNO055IMU imu;
    Orientation angles;
    VuforiaLocalizer vuforia;

    public String picturereading = null;

    double gs_previous_speed;
    double gs_previous_ticks_traveled;
    boolean gs_first_run = true;
    ElapsedTime gs_speed_timer = new ElapsedTime();

    private double wheel_encoder_ticks = 1440;
    private double wheel_diameter = 3.5;  // size of wheels
    public double ticks_per_inch = wheel_encoder_ticks / (wheel_diameter * 3.1416);
    private double liftencoderstartpos;

    public ElapsedTime runtime = new ElapsedTime();

    public void NAV_init() {
        leftWheel = hardwareMap.dcMotor.get("left");
        rightWheel = hardwareMap.dcMotor.get("right");
        jewelservo = hardwareMap.servo.get("js");
        sidejewelservo = hardwareMap.servo.get("sjs");
        leftclamp = hardwareMap.servo.get("lc");
        rightclamp = hardwareMap.servo.get("rc");
        liftmotor = hardwareMap.dcMotor.get("lm");
        relicmotor = hardwareMap.dcMotor.get("rm");

        jewelservo.setPosition(.6);
        sidejewelservo.setPosition(.2);

        leftclamp.setPosition(GlobalVarriables.leftclampinit);
        rightclamp.setPosition(GlobalVarriables.rightclampinit);

        rightWheel.setDirection(DcMotorSimple.Direction.FORWARD);
        leftWheel.setDirection(DcMotorSimple.Direction.REVERSE);

        leftWheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightWheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        relicmotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        liftmotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        liftmotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        BNO055IMU.Parameters IMUParameters = new BNO055IMU.Parameters();
        IMUParameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        IMUParameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        IMUParameters.calibrationDataFile = "BNO055IMUCalibration.json";

        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(IMUParameters);

        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        liftencoderstartpos = liftmotor.getCurrentPosition();

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);

        parameters.vuforiaLicenseKey = "AWaEPBn/////AAAAGWa1VK57tkUipP01PNk9ghlRuxjK1Oh1pmbHuRnpaJI0vi57dpbnIkpee7J1pQ2RIivfEFrobqblxS3dKUjRo52NMJab6Me2Yhz7ejs5SDn4G5dheW5enRNWmRBsL1n+9ica/nVjG8xvGc1bOBRsIeZyL3EZ2tKSJ407BRgMwNOmaLPBle1jxqAE+eLSoYsz/FuC1GD8c4S3luDm9Utsy/dM1W4dw0hDJFc+lve9tBKGBX0ggj6lpo9GUrTC8t19YJg58jsIXO/DiF09a5jlrTeB2LK+GndUDEGyZA1mS3yAR6aIBeDYnFw+79mVFIkTPk8wv3HIQfzoggCu0AwWJBVUVjkDxJOWfzCGjaHylZlo";

        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.FRONT;
        this.vuforia = ClassFactory.createVuforiaLocalizer(parameters);

        /*  This is instead of waitForStart so we can get telemetry if needed
        while (!isStarted()) {
            telemetry.update();
            idle();
        }
        */
        waitForStart();

    }

    public void turn_to_heading(double target_heading) {
        boolean go_right;
        double current_heading = currentheadingreading();
        double degrees_to_turn;
        double wheel_power;
        double prevheading = 0;
        ElapsedTime timeouttimer = new ElapsedTime();

        DbgLog.msg("10435 starting TURN_TO_HEADING");
        current_heading = currentheadingreading();
        degrees_to_turn = Math.abs(target_heading - current_heading);

        go_right = target_heading > current_heading;

        if (degrees_to_turn > 180) {
            go_right = !go_right;
            degrees_to_turn = 360 - degrees_to_turn;
        }

        timeouttimer.reset();
        prevheading = current_heading;
        while (degrees_to_turn > .5 && opModeIsActive() && runtime.seconds() < 26 && timeouttimer.seconds() < 2) {

            wheel_power = (2 * Math.pow((degrees_to_turn + 13) / 23, 2) + 19) / 100;

            if (go_right) {
                wheel_power = -wheel_power;
            }


            rightWheel.setPower(wheel_power);
            leftWheel.setPower(-wheel_power);

            current_heading = currentheadingreading();

            degrees_to_turn = Math.abs(target_heading - current_heading);       // Calculate how far is remaining to turn

            go_right = target_heading > current_heading;

            if (degrees_to_turn > 180) {
                go_right = !go_right;
                degrees_to_turn = 360 - degrees_to_turn;
            }

            if (Math.abs(current_heading - prevheading) > 1) {
                timeouttimer.reset();
                prevheading = current_heading;
            }

            DbgLog.msg("TURN_TO_HEADING" + " go right: " + go_right + " degrees to turn: " + degrees_to_turn + " wheel power: " + wheel_power + " current heading: " + current_heading + "Wheel power: " + Double.toString(wheel_power));
        }

        leftWheel.setPower(0);
        rightWheel.setPower(0);
        sleep(300);

        DbgLog.msg("10435 ending TURN_TO_HEADING" + Double.toString(target_heading) + "  Final heading:" + Double.toString(current_heading) + "  After set power 0:" + Double.toString(angles.firstAngle));

    } // end of turn_to_heading

    public void turn_to_heading_pirouette(double target_heading, boolean go_backwards) {
        boolean go_right;
        double current_heading;
        double degrees_to_turn;
        double prevheading = 0;
        double wheel_power = 0;
        ElapsedTime timeouttimer = new ElapsedTime();

        DbgLog.msg("10435 starting turn_to_heading_pirouette");

        current_heading = currentheadingreading();
        degrees_to_turn = Math.abs(target_heading - current_heading);

        go_right = target_heading > current_heading;

        if (degrees_to_turn > 180) {
            go_right = !go_right;
            degrees_to_turn = 360 - degrees_to_turn;
        }

        timeouttimer.reset();
        prevheading = current_heading;
        while (degrees_to_turn > .5 && opModeIsActive() && runtime.seconds() < 26 && timeouttimer.seconds() < 2) {

            wheel_power = (2 * Math.pow((degrees_to_turn + 13) / 23, 2) + 21) / 100;

            if (!go_right) {
                if (go_backwards) {
                    leftWheel.setPower(-wheel_power);
                    rightWheel.setPower(.2);
                } else {
                    rightWheel.setPower(wheel_power);
                    leftWheel.setPower(-.2);
                }
            } else {
                if (go_backwards) {
                    rightWheel.setPower(-wheel_power);
                    leftWheel.setPower(.2);
                } else {
                    leftWheel.setPower(wheel_power);
                    rightWheel.setPower(-.2);
                }
            }

            current_heading = currentheadingreading();                                // get the new current reading
            degrees_to_turn = Math.abs(target_heading - current_heading);       // Calculate how far is remaining to turn

            go_right = target_heading > current_heading;

            if (degrees_to_turn > 180) {
                go_right = !go_right;
                degrees_to_turn = 360 - degrees_to_turn;
            }

            if (Math.abs(current_heading - prevheading) > 1) {
                timeouttimer.reset();
                prevheading = current_heading;
            }

        }

        leftWheel.setPower(0);
        rightWheel.setPower(0);
        sleep(300);

        DbgLog.msg("10435 ending turn_to_heading " + Double.toString(target_heading) + "  Attempted heading: " + Double.toString(current_heading) + "  Real current Heading: " + Double.toString(currentheadingreading()) + "Wheel power: " + Double.toString(wheel_power));

    } // end of turn_to_heading_pirouette

    public void go_forward(double inches_to_travel, int heading, double speed, boolean runtimeoveride) {

        DbgLog.msg("10435 starting GO_FORWARD inches:" + Double.toString(inches_to_travel) + " heading:" + Integer.toString(heading) + " speed:" + Double.toString(speed));

        ElapsedTime log_timer = new ElapsedTime();

        double current_speed = .05;
        int ticks_to_travel;
        boolean destination_reached = false;
        boolean going_backwards = false;
        boolean runtimereached = false;
        double speed_increase = .05;
        double actual_speed;
        double lagreduction = 1.125;
        int start_position_L;
        int start_position_R;
        int previous_ticks_traveled_L = 0;
        int ticks_traveled_L;
        int ticks_traveled_R;
        int lowest_ticks_traveled;
        double remaining_inches = 0;
        double previous_log_timer = 0;
        double power_adjustment;

        ElapsedTime timeouttimer = new ElapsedTime();

        if (speed < 0) {
            inches_to_travel = inches_to_travel * 1.08;
            speed_increase = -speed_increase;
            current_speed = -current_speed;
            going_backwards = true;
        }

        inches_to_travel = inches_to_travel - lagreduction;

        ticks_to_travel = (int) (inches_to_travel * ticks_per_inch);

        start_position_L = leftWheel.getCurrentPosition();
        start_position_R = rightWheel.getCurrentPosition();

        log_timer.reset();
        timeouttimer.reset();

        gs_first_run = true;

        while (opModeIsActive() && !destination_reached && !runtimereached && timeouttimer.seconds() < 2) {

            if (runtime.seconds() > 26 && !runtimeoveride) {
                runtimereached = true;
            }

            current_speed = current_speed + speed_increase;  // this is to slowly ramp up the speed so we don't slip
            if (Math.abs(current_speed) > Math.abs(speed)) {
                current_speed = speed;
            }

            power_adjustment = go_straight_adjustment(heading);

            rightWheel.setPower(current_speed + power_adjustment);
            leftWheel.setPower(current_speed - power_adjustment);

            ticks_traveled_L = Math.abs(leftWheel.getCurrentPosition() - start_position_L);
            ticks_traveled_R = Math.abs(rightWheel.getCurrentPosition() - start_position_R);
            lowest_ticks_traveled = ticks_traveled_L;
            if (ticks_traveled_R < ticks_traveled_L) {
                lowest_ticks_traveled = ticks_traveled_R;
            }

            actual_speed = getSpeed(lowest_ticks_traveled);

            if (actual_speed > 0.05) {  // if we're going less than this we aren't moving.
                timeouttimer.reset();
            }

            telemetry.addData("GO_FORWARD ticks_to_travel", ticks_to_travel);
            telemetry.addData("actual speed:", actual_speed);
            telemetry.update();

            if (ticks_traveled_L != previous_ticks_traveled_L && log_timer.seconds() - previous_log_timer > .1) {
                DbgLog.msg("10435 GO_FORWARD ticks_traveled: L:" + Double.toString(ticks_traveled_L)
                        + " R:" + Double.toString(ticks_traveled_R) + " actual_speed:" + actual_speed + " current speed:" + current_speed + " speed:" + speed);
                previous_log_timer = log_timer.seconds();
                previous_ticks_traveled_L = ticks_traveled_L;
            }

            destination_reached = (lowest_ticks_traveled >= ticks_to_travel);

            remaining_inches = inches_to_travel - ((double) lowest_ticks_traveled / ticks_per_inch);

            if (remaining_inches <= actual_speed && Math.abs(speed) > .2) {
                speed = .2;
                if (going_backwards) {
                    speed = -speed;
                }
                DbgLog.msg("10435 GO_FORWARD slowing down: remaining_inches:" + Double.toString(remaining_inches)
                        + " lowest_ticks_traveled:" + Integer.toString(lowest_ticks_traveled));
            }

        }
        rightWheel.setPower(0);
        leftWheel.setPower(0);


        sleep(100);
        DbgLog.msg("10435 ending GO_FORWARD: opModeIsActive:" + Boolean.toString(opModeIsActive())
                + " distance traveled L:" + Double.toString((leftWheel.getCurrentPosition() - start_position_L) / ticks_per_inch)
                + " distance traveled R:" + Double.toString((rightWheel.getCurrentPosition() - start_position_R) / ticks_per_inch)
                + " destination_reached:" + Boolean.toString(destination_reached)
                + " runtimer reached:" + Boolean.toString(runtimereached)
                + " timouttimer:" + Double.toString(timeouttimer.seconds()));

    } // end of go_forward

    public String[] vuforia_scan() {

        DbgLog.msg("10435 Starting Vuforia_Scan");

        ElapsedTime vuforiascantime = new ElapsedTime();

        double redcount = 0;
        double bluecount = 0;
        String rightjewelcolor;
        boolean picturefound = false;

        VuforiaTrackables relicTrackables = this.vuforia.loadTrackablesFromAsset("RelicVuMark");
        VuforiaTrackable relicTemplate = relicTrackables.get(0);
        relicTrackables.activate();

        vuforiascantime.reset();
        while (opModeIsActive() && !picturefound && vuforiascantime.seconds() < 5) {
            RelicRecoveryVuMark vuMark = RelicRecoveryVuMark.from(relicTemplate);

            if (vuMark != RelicRecoveryVuMark.UNKNOWN) {
                picturereading = vuMark.toString();
                telemetry.addData("Vumark visible:", picturereading);
                picturefound = true;
            } else {
                telemetry.addLine("VuMark not visible");
            }
            telemetry.update();
        }

        DbgLog.msg("10435 Vuforia_Scan: " + "picturereading:" + picturereading);

        Image rgbImage = null;

        Vuforia.setFrameFormat(PIXEL_FORMAT.RGB565, true);
        VuforiaLocalizer.CloseableFrame closeableframe = null;
        this.vuforia.setFrameQueueCapacity(6);

        try {
            closeableframe = this.vuforia.getFrameQueue().take();
            long numImages = closeableframe.getNumImages();

            for (int i = 0; i < numImages; i++) {
                if (closeableframe.getImage(i).getFormat() == PIXEL_FORMAT.RGB565) {
                    rgbImage = closeableframe.getImage(i);
                    break;
                }
            }
        } catch (InterruptedException exc) {
            DbgLog.msg("10435 Vuforia_Scan: closeableframe exception" + exc.toString() );
            return null;
        } finally {
            if (closeableframe != null) closeableframe.close();
        }

        if (rgbImage != null) {

            Bitmap bm = Bitmap.createBitmap(rgbImage.getWidth(), rgbImage.getHeight(), Bitmap.Config.RGB_565);
            bm.copyPixelsFromBuffer(rgbImage.getPixels());
            int bmwidth;
            int bmheight;
            int onepixel;
            int redbluediv = 100000;

            bmheight = bm.getHeight();
            bmwidth = bm.getWidth();

            int iheight, jwidth;
            for (iheight = (int) (bmheight * .6); iheight < bmheight; ++iheight) {  // starting at .6 because looking at bottom 40% of image
                for (jwidth = 0; jwidth < bmwidth * .3; ++jwidth) {  // looking at first 30% of image which is right side because image is backward
                    onepixel = bm.getPixel(jwidth, iheight);
                    redcount += Color.red(onepixel);
                    bluecount += Color.blue(onepixel);
                }
            }

            //saves the bitmap as a file
            String path = Environment.getExternalStorageDirectory().toString();
            FileOutputStream out = null;
            try {
                File file = new File(path, "Bitmap.png");
                out = new FileOutputStream(file);
                bm.compress(Bitmap.CompressFormat.PNG, 100, out);
            } catch (Exception e) {
                DbgLog.msg("10435 Vuforia_Scan: FileOutputStream exception" + e.toString() );

                e.printStackTrace();
            } finally {
                try {
                    if (out != null) {
                        out.close();
                    }
                } catch (IOException e) {
                    DbgLog.msg("10435 Vuforia_Scan: FileOutputStream close exception" + e.toString() );
                    e.printStackTrace();
                }
            }


            redcount = redcount / redbluediv;
            bluecount = bluecount / redbluediv;
            telemetry.addData("Red count:", redcount);
            telemetry.addData("Blue count:", bluecount);
            DbgLog.msg("10435 Vuforia_Scan:"
                    + " Red Count:" + Double.toString(redcount)
                    + " Blue Count:" + Double.toString(bluecount)
                    + " ImageXsize:" + Integer.toString(bm.getWidth())
                    + " ImageYsize:" + Integer.toString(bm.getHeight())
            );
        } else {
            DbgLog.msg("10435 Vuforia_Scan: rgbImage = null");
        }

        if (redcount > bluecount) {
            rightjewelcolor = "red";
        } else {
            rightjewelcolor = "blue";
        }

        if (redcount + bluecount == 0){
            rightjewelcolor = "none";
        }

        String returnvalue[] = new String[2];
        returnvalue[0] = picturereading;
        returnvalue[1] = rightjewelcolor;

        telemetry.update();

        DbgLog.msg("10435 Ending Vuforia_Scan");

        return returnvalue;
    }

    public void jewelknockvuforia(String RoB, String rightjewelcolor, boolean useside) {

        DbgLog.msg("10435 Starting Jewel Knock Vuforia");
        DbgLog.msg("10435 Right Jewel Color: " + rightjewelcolor);

        if (rightjewelcolor != "none") {
            if (useside) {
                sidejewelservo.setPosition(.97);
            } else {
                jewelservo.setPosition(0);
            }
            sleep(1000);

            if (RoB == "red" && rightjewelcolor == "blue" || RoB == "blue" && rightjewelcolor == "red") {
                DbgLog.msg("10435 Jewel Knock Vuforia: Turning Left");
                turn_to_heading(352);
            } else if (RoB == "red" && rightjewelcolor == "red" || RoB == "blue" && rightjewelcolor == "blue") {
                DbgLog.msg("10435 Jewel Knock Vuforia: Turning Right");
                turn_to_heading(8);
            }

            if (useside) {
                sidejewelservo.setPosition(.2);
            } else {
                jewelservo.setPosition(.6);
            }
            sleep(200);
            turn_to_heading(0);
        }

        DbgLog.msg("10435 Ending Jewel Knock Vuforia");
    }
/*
    public void jewelknock(String RoB) {

        DbgLog.msg("10435 Staring JEWEL_KNOCK");

        int redreadingright = 0;
        int redreadingleft = 0;
        int bluereadingright = 0;
        int bluereadingleft = 0;
        int alphareadingright = 0;
        int alphareadingleft = 0;
        int timeslooped = 0;
        int leftreddiff;
        int rightreddiff;
        boolean redright;
        boolean redleft;

        jewelservo.setPosition(0);
        sleep(2000);

        // wait up to 3 seconds for servo arm to go down and to get a color reading if we don't get a color reading then move on
        while (opModeIsActive() && timeslooped < 3) {
            sleep(10);
            timeslooped = timeslooped + 1;
            redreadingright = redreadingright + jewelcsright.red();
            bluereadingright = bluereadingright + jewelcsright.blue();
            alphareadingright = alphareadingright + jewelcsright.alpha();
            redreadingleft = redreadingleft + jewelcsleft.red();
            bluereadingleft = bluereadingleft + jewelcsleft.blue();
            alphareadingleft = alphareadingleft + jewelcsleft.alpha();
        }

        leftreddiff = redreadingleft - bluereadingleft;
        rightreddiff = redreadingright - bluereadingright;

        redright = rightreddiff > 0;
        redleft = leftreddiff > 0;

        if (redleft == redright) {
            if (Math.abs(leftreddiff) > Math.abs(rightreddiff)) {
                redright = !redright;
            }
        }  // we don't need to see if redleft is correct

        if (RoB == "red" && redright || RoB == "blue" && !redright) {
            turn_to_heading(8);
            jewelservo.setPosition(.6);
            sleep(200);
            turn_to_heading(0);
        } else {
            turn_to_heading(352);
            jewelservo.setPosition(.6);
            sleep(200);
            turn_to_heading(0);
        }

        jewelservo.setPosition(.6);
        sleep(400);

    }

    public void jewelknockside(String RoB) {

        DbgLog.msg("10435 Staring JEWEL_KNOCK_SIDE");

        double redreading = 0;
        double bluereading = 0;
        double redreadingadj = 0;
        double bluereadingadj = 0;
        double baseredreading = 0;
        double basebluereading = 0;
        int timeslooped = 0;
        String colorfound;

        sidejewelservo.setPosition(.77);

        sleep(1000);

        while (opModeIsActive() && timeslooped < 3) {
            sleep(10);
            timeslooped = timeslooped + 1;
            baseredreading = baseredreading + sidejewelcs.red();
            basebluereading = basebluereading + sidejewelcs.blue();
        }

        sidejewelservo.setPosition(.97);

        sleep(1000);

        timeslooped = 0;

        while (opModeIsActive() && timeslooped < 3) {
            sleep(10);
            timeslooped = timeslooped + 1;
            redreading = redreading + sidejewelcs.red();
            bluereading = bluereading + sidejewelcs.blue();
        }

        telemetry.addData("Base Red Reading", baseredreading);
        telemetry.addData("Red Reading", redreading);

        redreadingadj = redreading - baseredreading;

        telemetry.addData("Red Reading Diff", redreadingadj);
        telemetry.addData("Base Blue Reading", basebluereading);
        telemetry.addData("Blue Reading", bluereading);

        bluereadingadj = bluereading - basebluereading;

        telemetry.addData("Blue Reading Diff", bluereadingadj);

        if (redreadingadj - bluereadingadj > 15 || redreading - bluereading > 40) {
            colorfound = "red";
        } else if (bluereadingadj - redreadingadj > 8 || bluereading - redreading > 40) {
            colorfound = "blue";
        } else {
            colorfound = "none";
        }

        if (RoB == "red" && colorfound == "red" || RoB == "blue" && colorfound == "blue") {
            turn_to_heading(352);
            sidejewelservo.setPosition(.2);
            sleep(200);
            turn_to_heading(0);
        } else if (colorfound != "none") {
            turn_to_heading(8);
            sidejewelservo.setPosition(.2);
            sleep(200);
            turn_to_heading(0);
        }

        sidejewelservo.setPosition(.2);
        sleep(400);

        DbgLog.msg("10435 " + "Color Found: ", colorfound + "Red Reading Adjusted: ", redreadingadj + "Blue Reading Adjusted: ", bluereadingadj + "Blue Reading: ", bluereading + "Red Reading", redreading);

    }
*/
    public void lift_glyph(String UoD) {
        final double incheslifted = 6;
        double ticksperrevlift = 560;
        double inchesperrevlift = 5.375;
        double ticksperinchlift = ticksperrevlift / inchesperrevlift;
        double liftencoderpos;
        double lifttargetpos = 0;

        sleep(200);

        if (UoD == "up") {
            leftclamp.setPosition(GlobalVarriables.leftclampclosed);
            rightclamp.setPosition(GlobalVarriables.rightclampclosed);
            sleep(2000);
            lifttargetpos = incheslifted * ticksperinchlift;
        } else if (UoD == "down") {
            lifttargetpos = 0;
        }

        liftencoderpos = liftmotor.getCurrentPosition() - liftencoderstartpos;
        if (UoD == "down") {
            while (lifttargetpos < liftencoderpos - 40 && opModeIsActive()) {
                liftencoderpos = liftmotor.getCurrentPosition() - liftencoderstartpos;
                if (liftencoderpos - lifttargetpos > 120) {
                    liftmotor.setPower(-.8);
                } else {
                    liftmotor.setPower(-.3);
                }
            }
        } else if (UoD == "up") {
            while (lifttargetpos > liftencoderpos + 40 && opModeIsActive()) {
                liftencoderpos = liftmotor.getCurrentPosition() - liftencoderstartpos;
                if (lifttargetpos - liftencoderpos > 120) {
                    liftmotor.setPower(1);
                } else {
                    liftmotor.setPower(.6);
                }
            }
        }
        liftmotor.setPower(0);
    }

    private double getSpeed(double ticks_traveled) {
        double new_speed;

        if (gs_first_run) {
            gs_previous_ticks_traveled = ticks_traveled;
            gs_speed_timer.reset();
            gs_previous_speed = 1;
            gs_first_run = false;
        }

        if (gs_speed_timer.seconds() >= .1) {
            new_speed = (ticks_traveled - gs_previous_ticks_traveled) / 46.5;  // At max speed we travel about 4800 ticks in a second so this give a range of 0 - 10 for speed
            gs_speed_timer.reset();
            gs_previous_speed = new_speed;
            gs_previous_ticks_traveled = ticks_traveled;
            DbgLog.msg("10435 getspeed:" + Double.toString(new_speed));
        } else {
            new_speed = gs_previous_speed;
        }

        return new_speed;
    }

    private double currentheadingreading() {
        double current_heading;
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        current_heading = angles.firstAngle;

        if (current_heading < 0) {
            current_heading = -current_heading;
        } else {
            current_heading = 360 - current_heading;
        }

        return current_heading;
    }

    private double go_straight_adjustment(double target_heading) {

        //  This function outputs power_adjustment that should be added to right wheel and subtracted from left wheel

        double gs_adjustment;
        double current_heading;
        double degrees_off;
        boolean go_right;

        DbgLog.msg("10435 starting go_straight_adjustment heading:" + Double.toString(target_heading) + " current heading:" + Double.toString(angles.firstAngle));

        current_heading = currentheadingreading();

        go_right = target_heading > current_heading;
        degrees_off = Math.abs(target_heading - current_heading);

        if (degrees_off > 180) {
            go_right = !go_right;
            degrees_off = 360 - degrees_off;
        }

        if (degrees_off < .3) {
            gs_adjustment = 0;
        } else {
            gs_adjustment = (Math.pow((degrees_off + 2) / 5, 2) + 10) / 100;
        }

        if (go_right) {
            gs_adjustment = -gs_adjustment;
        }

        DbgLog.msg("10435 go_straight_adjustment adjustment:" + Double.toString(gs_adjustment));

        return gs_adjustment;

    } // end of go_straight_adjustment
}


