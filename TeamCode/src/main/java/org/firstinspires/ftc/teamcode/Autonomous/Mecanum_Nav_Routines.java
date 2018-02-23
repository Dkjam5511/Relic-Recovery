package org.firstinspires.ftc.teamcode.Autonomous;

import android.graphics.Bitmap;
import android.graphics.Color;
import android.graphics.Matrix;
import android.os.Environment;
import android.provider.Settings;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.vuforia.Image;
import com.vuforia.PIXEL_FORMAT;
import com.vuforia.Vuforia;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
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
 * Created by Drew on 1/10/2018.
 */

abstract public class Mecanum_Nav_Routines extends LinearOpMode {

    DcMotor leftFront;
    DcMotor rightFront;
    DcMotor leftRear;
    DcMotor rightRear;
    DcMotor liftmotor;
    public Servo leftclamp;
    public Servo rightclamp;
    Servo leftjewelservo;
    Servo leftjewelservoflipper;
    Servo rightjewelservo;
    Servo rightjewelservoflipper;
    Servo leftglyphwheel;
    Servo rightglyphwheel;
    ModernRoboticsI2cRangeSensor rangesensor;
    ColorSensor cs;
    BNO055IMU imu;
    Orientation angles;
    VuforiaLocalizer vuforia;

    public String picturereading = null;

    double gs_previous_speed;
    double gs_previous_ticks_traveled;
    boolean gs_first_run = true;
    ElapsedTime gs_speed_timer = new ElapsedTime();

    private double wheel_encoder_ticks = 537.6;
    private double wheel_diameter = 3.75;  // size of wheels
    public double ticks_per_inch = wheel_encoder_ticks / (wheel_diameter * 3.1416);
    private double liftencoderstartpos;

    public ElapsedTime runtime = new ElapsedTime();

    public void MNav_Init() {
        leftFront = hardwareMap.dcMotor.get("lf");
        rightFront = hardwareMap.dcMotor.get("rf");
        leftRear = hardwareMap.dcMotor.get("lr");
        rightRear = hardwareMap.dcMotor.get("rr");
        leftclamp = hardwareMap.servo.get("lc");
        rightclamp = hardwareMap.servo.get("rc");
        liftmotor = hardwareMap.dcMotor.get("lm");
        leftjewelservo = hardwareMap.servo.get("ljs");
        leftjewelservoflipper = hardwareMap.servo.get("ljsf");
        rightjewelservo = hardwareMap.servo.get("rjs");
        rightjewelservoflipper = hardwareMap.servo.get("rjsf");
        leftglyphwheel = hardwareMap.servo.get("lgw");
        rightglyphwheel = hardwareMap.servo.get("rgw");
        rangesensor = hardwareMap.get(ModernRoboticsI2cRangeSensor.class, "rs");
        cs = hardwareMap.colorSensor.get("cs");

        rightFront.setDirection(DcMotor.Direction.REVERSE);
        rightRear.setDirection(DcMotor.Direction.REVERSE);

        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        liftmotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        BNO055IMU.Parameters IMUParameters = new BNO055IMU.Parameters();
        IMUParameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        IMUParameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        IMUParameters.calibrationDataFile = "BNO055IMUCalibration.json";

        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(IMUParameters);

        leftclamp.setPosition(GlobalVarriables.leftclampinitpos);
        rightclamp.setPosition(GlobalVarriables.rightclampinitpos);

        rightjewelservo.setPosition(GlobalVarriables.rightjewelservoinit);
        leftjewelservo.setPosition(GlobalVarriables.leftjewelservoinit);
        rightjewelservoflipper.setPosition(GlobalVarriables.rightjewelservoflipperinit);
        leftjewelservoflipper.setPosition(GlobalVarriables.leftjewelservoflipperinit);

        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        liftencoderstartpos = liftmotor.getCurrentPosition();

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);

        parameters.vuforiaLicenseKey = "AWaEPBn/////AAAAGWa1VK57tkUipP01PNk9ghlRuxjK1Oh1pmbHuRnpaJI0vi57dpbnIkpee7J1pQ2RIivfEFrobqblxS3dKUjRo52NMJab6Me2Yhz7ejs5SDn4G5dheW5enRNWmRBsL1n+9ica/nVjG8xvGc1bOBRsIeZyL3EZ2tKSJ407BRgMwNOmaLPBle1jxqAE+eLSoYsz/FuC1GD8c4S3luDm9Utsy/dM1W4dw0hDJFc+lve9tBKGBX0ggj6lpo9GUrTC8t19YJg58jsIXO/DiF09a5jlrTeB2LK+GndUDEGyZA1mS3yAR6aIBeDYnFw+79mVFIkTPk8wv3HIQfzoggCu0AwWJBVUVjkDxJOWfzCGjaHylZlo";

        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.FRONT;
        this.vuforia = ClassFactory.createVuforiaLocalizer(parameters);

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

//            wheel_power = (Math.pow((degrees_to_turn + 5) / 40, 2) + 2) / 100;

            wheel_power = (2 * Math.pow((degrees_to_turn + 13) / 30, 2) + 15) / 100;

            if (go_right) {
                wheel_power = -wheel_power;
            }

            rightFront.setPower(wheel_power);
            rightRear.setPower(wheel_power);
            leftFront.setPower(-wheel_power);
            leftRear.setPower(-wheel_power);

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

        rightFront.setPower(0);
        rightRear.setPower(0);
        leftFront.setPower(0);
        leftRear.setPower(0);

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

            wheel_power = (2 * Math.pow((degrees_to_turn + 13) / 23, 2) + 10) / 100;

            if (!go_right) {
                if (go_backwards) {
                    leftRear.setPower(-wheel_power * .8);
                    leftFront.setPower(-wheel_power * 1.4);
                    rightFront.setPower(wheel_power);
                    rightRear.setPower(0);
                } else {
                    rightRear.setPower(wheel_power * .8);
                    rightFront.setPower(wheel_power * 1.4);
                    leftFront.setPower(-wheel_power);
                    leftRear.setPower(0);
                }
            } else {
                if (go_backwards) {
                    rightRear.setPower(-wheel_power * .8);
                    rightFront.setPower(-wheel_power * 1.4);
                    leftFront.setPower(wheel_power);
                    leftRear.setPower(0);
                } else {
                    leftRear.setPower(wheel_power * .8);
                    leftFront.setPower(wheel_power * 1.4);
                    rightFront.setPower(-wheel_power);
                    rightRear.setPower(-0);
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

        leftRear.setPower(0);
        rightRear.setPower(0);
        leftFront.setPower(0);
        rightFront.setPower(0);
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
        double lagreduction = 2.125;
        int start_position_l_Front;
        int start_position_l_Rear;
        int start_position_r_Front;
        int start_position_r_Rear;
        int previous_ticks_traveled_L = 0;
        int ticks_traveled_l_Front;
        int ticks_traveled_l_Rear;
        int ticks_traveled_r_Front;
        int ticks_traveled_r_Rear;
        int lowest_ticks_traveled_l = 0;
        int lowest_ticks_traveled_r = 0;
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

        start_position_l_Front = leftFront.getCurrentPosition();
        start_position_l_Rear = leftRear.getCurrentPosition();
        start_position_r_Front = rightFront.getCurrentPosition();
        start_position_r_Rear = rightRear.getCurrentPosition();

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

            rightFront.setPower(current_speed + power_adjustment);
            rightRear.setPower(current_speed + power_adjustment);
            leftFront.setPower(current_speed - power_adjustment);
            leftRear.setPower(current_speed - power_adjustment);

            ticks_traveled_l_Front = Math.abs(leftFront.getCurrentPosition() - start_position_l_Front);
            ticks_traveled_l_Rear = Math.abs(leftRear.getCurrentPosition() - start_position_l_Rear);
            ticks_traveled_r_Front = Math.abs(rightFront.getCurrentPosition() - start_position_r_Front);
            ticks_traveled_r_Rear = Math.abs(rightRear.getCurrentPosition() - start_position_r_Rear);

            lowest_ticks_traveled_l = Math.min(ticks_traveled_l_Front, ticks_traveled_l_Rear);
            lowest_ticks_traveled_r = Math.min(ticks_traveled_r_Front, ticks_traveled_r_Rear);
            lowest_ticks_traveled = Math.min(lowest_ticks_traveled_l, lowest_ticks_traveled_r);

            actual_speed = getSpeed(lowest_ticks_traveled);

            if (actual_speed > 0.05) {  // if we're going less than this we aren't moving.
                timeouttimer.reset();
            }

            telemetry.addData("GO_FORWARD ticks_to_travel", ticks_to_travel);
            telemetry.addData("actual speed:", actual_speed);
            telemetry.update();

            if (lowest_ticks_traveled_l != previous_ticks_traveled_L && log_timer.seconds() - previous_log_timer > .1) {
                DbgLog.msg("10435 GO_FORWARD ticks_traveled: L:" + Double.toString(lowest_ticks_traveled_l)
                        + " R:" + Double.toString(lowest_ticks_traveled_r) + " actual_speed:" + actual_speed + " current speed:" + current_speed + " speed:" + speed);
                previous_log_timer = log_timer.seconds();
                previous_ticks_traveled_L = lowest_ticks_traveled_l;
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
        rightFront.setPower(0);
        rightRear.setPower(0);
        leftFront.setPower(0);
        leftRear.setPower(0);


        sleep(100);
        DbgLog.msg("10435 ending GO_FORWARD: opModeIsActive:" + Boolean.toString(opModeIsActive())
                + " distance traveled L:" + Double.toString((lowest_ticks_traveled_l / ticks_per_inch))
                + " distance traveled R:" + Double.toString((lowest_ticks_traveled_r / ticks_per_inch))
                + " destination_reached:" + Boolean.toString(destination_reached)
                + " runtimer reached:" + Boolean.toString(runtimereached)
                + " timouttimer:" + Double.toString(timeouttimer.seconds()));

    } // end of go_forward

    public void go_sideways(String RoB, double angledegrees, double heading, double power, double maxtime, double walldistance) {

        DbgLog.msg("10435 Starting go_sideways"
                + " angledegrees:" + Double.toString(angledegrees)
                + " heading:" + Double.toString(heading)
                + " power:" + Double.toString(power)
                + " maxtime:" + Double.toString(maxtime)
        );

        ElapsedTime timerun = new ElapsedTime();
        double stickpower = power;
        double angleradians;
        double leftfrontpower;
        double rightfrontpower;
        double leftrearpower;
        double rightrearpower;
        double turningpower = 0;
        double inchesreadfromwall;
        double poweradjustment = 0;
        double walldistancesensitivity = 3;
        double colorreading = 0;

        boolean colorfound = false;


        // For the cos and sin calculations below in the mecanum power calcs, angleradians = 0 is straight to the right and 180 is straight to the left.
        // Negative numbers up to -180 are backward.  Postive numbers up to 180 are forward.
        // We subtract 90 from it then convert degrees to radians because *our* robot code thinks of 0 degrees as forward, 90 as right, 180 as backward, 270 as left.

        // This converts from *our* degrees to radians used by the mecanum power calcs.
        // Upper left quadrant (degrees > 270) is special because in that quadrant as our degrees goes up, radians goes down.
        if (angledegrees < 270) {
            angleradians = ((angledegrees - 90) * -1) * Math.PI / 180;
        } else {
            angleradians = (450 - angledegrees) * Math.PI / 180;
        }

        angleradians = angleradians - Math.PI / 4; //adjust by 45 degrees for the mecanum wheel calculations below

        while (timerun.seconds() < maxtime && runtime.seconds() < 26 && opModeIsActive() && !colorfound) {

            inchesreadfromwall = rangesensorreading();
            if (RoB == "red" | RoB == "blue") {
                if (RoB == "red") {
                    colorreading = cs.red();
                } else {
                    colorreading = cs.blue();
                }
                if (colorreading >= 29 && RoB == "red") {
                    colorfound = true;
                } else if (colorreading >= 17 && RoB == "blue") {
                    colorfound = true;
                }
            }

            if (Math.abs(walldistance - inchesreadfromwall) < .4) {
                walldistancesensitivity = 1.5;
            }

            if (walldistance > 0 && (Math.abs(walldistance - inchesreadfromwall) < walldistancesensitivity)) {
                poweradjustment = (inchesreadfromwall - walldistance) / 30;
            } else {
                poweradjustment = 0;
            }

            turningpower = -go_straight_adjustment(heading) * .6;

            leftfrontpower = stickpower * Math.cos(angleradians) + turningpower + poweradjustment;
            rightfrontpower = stickpower * Math.sin(angleradians) - turningpower + poweradjustment;
            leftrearpower = stickpower * Math.sin(angleradians) + turningpower + poweradjustment;
            rightrearpower = stickpower * Math.cos(angleradians) - turningpower + poweradjustment;

            leftFront.setPower(leftfrontpower);
            rightFront.setPower(rightfrontpower);
            leftRear.setPower(leftrearpower);
            rightRear.setPower(rightrearpower);

            telemetry.addData("Inches Read From Wall", inchesreadfromwall);
            telemetry.addData("Color Reading", colorreading);
            telemetry.addData("Wall Distance", walldistance);
            telemetry.addData("Power Adjustment", poweradjustment);
            telemetry.update();

            DbgLog.msg("10435 inchesreadfromwall:" + Double.toString(inchesreadfromwall)
                    + " walldistance:" + Double.toString(walldistance)
                    + " poweradjustment:" + Double.toString(poweradjustment)
                    + "color reading" + Double.toString(colorreading)
            );

        }

        leftFront.setPower(0);
        rightFront.setPower(0);
        leftRear.setPower(0);
        rightRear.setPower(0);

        sleep(1000);
    }

    public void wall_distance_align(double walldistance) {

        DbgLog.msg("10435 Starting Wall Distance Align");

        ElapsedTime timeouttimer = new ElapsedTime();

        double inchesreadfromwall;
        double poweradjustment;
        boolean tooclose = false;

        inchesreadfromwall = rangesensorreading();

        timeouttimer.reset();

        while (Math.abs(walldistance - inchesreadfromwall) > .5 && timeouttimer.seconds() < 2 && !tooclose && opModeIsActive()) {
            inchesreadfromwall = rangesensorreading();
            poweradjustment = (Math.pow(Math.abs(inchesreadfromwall - walldistance) / 9, 3) + 15) / 100;
            if (inchesreadfromwall - walldistance < 0){
                poweradjustment = -poweradjustment;
            }

            if (walldistance - inchesreadfromwall > 5) {
                tooclose = true;
            }

            DbgLog.msg("10435 Wall Distance Align:"
                    + " Inches Read From Wall:" + Double.toString(inchesreadfromwall)
                    + " poweradjustment:" + Double.toString(poweradjustment)
            );

            leftFront.setPower(poweradjustment);
            rightFront.setPower(poweradjustment);
            leftRear.setPower(poweradjustment);
            rightRear.setPower(poweradjustment);
        }

        leftFront.setPower(0);
        rightFront.setPower(0);
        leftRear.setPower(0);
        rightRear.setPower(0);

    }

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
                    if (rgbImage != null) {
                        break;
                    }
                }
            }
        } catch (InterruptedException exc) {
            DbgLog.msg("10435 Vuforia_Scan: closeableframe exception" + exc.toString());
            return null;
        } finally {
            if (closeableframe != null) closeableframe.close();
        }

        if (rgbImage != null) {

            // copy the bitmap from the Vuforia frame
            Bitmap croppedbm = Bitmap.createBitmap(rgbImage.getWidth(), rgbImage.getHeight(), Bitmap.Config.RGB_565);
            croppedbm.copyPixelsFromBuffer(rgbImage.getPixels());
/*
            //saves the bitmap as a file
            String path = Environment.getExternalStorageDirectory().toString();
            FileOutputStream out = null;
            try {
                File file = new File(path, "Bitmap.png");
                out = new FileOutputStream(file);
                croppedbm.compress(Bitmap.CompressFormat.PNG, 100, out);
            } catch (Exception e) {
                DbgLog.msg("10435 Vuforia_Scan: FileOutputStream exception" + e.toString());
                e.printStackTrace();
            } finally {
                try {
                    if (out != null) {
                        out.close();
                    }
                } catch (IOException e) {
                    DbgLog.msg("10435 Vuforia_Scan: FileOutputStream close exception" + e.toString());
                    e.printStackTrace();
                }
            }
*/
            DbgLog.msg("10435 Vuforia_Scan:"
                    + " Original Width:" + Integer.toString(croppedbm.getWidth())
                    + " Original Height:" + Integer.toString(croppedbm.getHeight()));

            //** rotate 90 degrees **//
            //Matrix matrix = new Matrix();
            //matrix.postRotate(90);
            //bm = Bitmap.createBitmap(bm, 0, 0, bmwidth, bmheight, matrix, true);

            // Crop the image
            int croppedwidth = 400;
            int croppedheight = 360;
            int croppedxstart = croppedbm.getWidth() - croppedwidth;
            // Ok now do the cropping, and put it back in same buffer.
            // The image is rotated 90 to the left and 0,0 is the upper left
            croppedbm = Bitmap.createBitmap(croppedbm, croppedxstart, 0, croppedwidth, croppedheight);

            // Add up the red and blue in the pixels
            int onepixel;
            int iheight, jwidth;
            for (iheight = 0; iheight < croppedheight; ++iheight) {
                for (jwidth = 0; jwidth < croppedwidth; ++jwidth) {
                    onepixel = croppedbm.getPixel(jwidth, iheight);
                    redcount += Color.red(onepixel);
                    bluecount += Color.blue(onepixel);
                }
            }
/*
            //saves the bitmap as a file
            try {
                File file = new File(path, "CroppedBitmap.png");
                out = new FileOutputStream(file);
                croppedbm.compress(Bitmap.CompressFormat.PNG, 100, out);
            } catch (Exception e) {
                DbgLog.msg("10435 Vuforia_Scan: FileOutputStream exception" + e.toString());
                e.printStackTrace();
            } finally {
                try {
                    if (out != null) {
                        out.close();
                    }
                } catch (IOException e) {
                    DbgLog.msg("10435 Vuforia_Scan: FileOutputStream close exception" + e.toString());
                    e.printStackTrace();
                }
            }
*/
            int redbluediv = 100000;
            redcount = redcount / redbluediv;
            bluecount = bluecount / redbluediv;
            telemetry.addData("Red count:", redcount);
            telemetry.addData("Blue count:", bluecount);
            DbgLog.msg("10435 Vuforia_Scan:"
                    + " Red Count:" + Double.toString(redcount)
                    + " Blue Count:" + Double.toString(bluecount)
                    + " Cropped Width:" + Integer.toString(croppedbm.getWidth())
                    + " Cropped Height:" + Integer.toString(croppedbm.getHeight())
            );
        } else {
            DbgLog.msg("10435 Vuforia_Scan: Vuforia failure - rgbImage = null");
        }

        if (redcount > bluecount) {
            rightjewelcolor = "red";
        } else {
            rightjewelcolor = "blue";
        }

        if (redcount + bluecount == 0) {
            rightjewelcolor = "none";
        }

        String returnvalue[] = new String[2];
        returnvalue[0] = picturereading;
        returnvalue[1] = rightjewelcolor;

        telemetry.update();

        DbgLog.msg("10435 Ending Vuforia_Scan");

        return returnvalue;
    }

    public void lift_glyph(String UoD, double inchestolift, boolean doclamp) {
        double ticksperrevlift = 560;
        double inchesperrevlift = 5.375;
        double ticksperinchlift = ticksperrevlift / inchesperrevlift;
        double liftencoderpos;
        double lifttargetpos = 0;

        if (doclamp) {
            leftclamp.setPosition(GlobalVarriables.leftclampclosedpos);
            rightclamp.setPosition(GlobalVarriables.rightclampclosedpos);
            sleep(1200);
        }

        lifttargetpos = inchestolift * ticksperinchlift;
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

    public void jewelknockvuforia(String RoB, String rightjewelcolor, boolean useright) {

        DbgLog.msg("10435 Starting Jewel Knock Vuforia");
        DbgLog.msg("10435 Right Jewel Color: " + rightjewelcolor);

        if (rightjewelcolor != "none") {
            if (useright) {
                rightjewelservo.setPosition(.17);
            } else {
                leftjewelservo.setPosition(.83);
            }
            sleep(700);

            if (RoB == "red" && rightjewelcolor == "blue" || RoB == "blue" && rightjewelcolor == "red") {
                DbgLog.msg("10435 Jewel Knock Vuforia: Hitting Left");
                if (useright) {
                    rightjewelservoflipper.setPosition(GlobalVarriables.rightjewelservoflipperinit - GlobalVarriables.jewelflipperswing);
                } else {
                    leftjewelservoflipper.setPosition(GlobalVarriables.leftjewelservoflipperinit - GlobalVarriables.jewelflipperswing);
                }
            } else if (RoB == "red" && rightjewelcolor == "red" || RoB == "blue" && rightjewelcolor == "blue") {
                DbgLog.msg("10435 Jewel Knock Vuforia: Hitting Right");
                if (useright) {
                    rightjewelservoflipper.setPosition(GlobalVarriables.rightjewelservoflipperinit + GlobalVarriables.jewelflipperswing);
                } else {
                    leftjewelservoflipper.setPosition(GlobalVarriables.leftjewelservoflipperinit + GlobalVarriables.jewelflipperswing);
                }
            }

            sleep(700);
            rightjewelservo.setPosition(GlobalVarriables.rightjewelservoinit);
            leftjewelservo.setPosition(GlobalVarriables.leftjewelservoinit);
            rightjewelservoflipper.setPosition(GlobalVarriables.rightjewelservoflipperinit);
            leftjewelservoflipper.setPosition(GlobalVarriables.leftjewelservoflipperinit);
            sleep(200);

        }

        DbgLog.msg("10435 Ending Jewel Knock Vuforia");
    }

    public void setwheelintake(boolean in, boolean on){
        if (on) {
            if (in) {
                leftglyphwheel.setPosition(1);
                rightglyphwheel.setPosition(0);
            } else {
                leftglyphwheel.setPosition(0);
                rightglyphwheel.setPosition(1);
            }
        } else {
            leftglyphwheel.setPosition(.5);
            rightglyphwheel.setPosition(.5);
        }
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
            gs_adjustment = (Math.pow((degrees_off + 2) / 5, 2) + 2) / 100;
        }

        if (go_right) {
            gs_adjustment = -gs_adjustment;
        }

        DbgLog.msg("10435 go_straight_adjustment adjustment:" + Double.toString(gs_adjustment));

        return gs_adjustment;

    } // end of go_straight_adjustment

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

    private double rangesensorreading() {
        double inchesfound;
        inchesfound = rangesensor.getDistance(DistanceUnit.INCH);

        return inchesfound;
    }

}

