package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;

/**
 * Created by Drew on 9/17/2017.
 * :D
 */

public abstract class Navigation_Routines extends LinearOpMode {

    DcMotor leftWheel;
    DcMotor rightWheel;
    BNO055IMU imu;
    Orientation angles;
    Velocity speeds;
    VuforiaLocalizer vuforia;

    public String picturereading = null;

    private double wheel_encoder_ticks = 288;
    private double wheel_diameter = 3.53;  // size of wheels
    public double ticks_per_inch = wheel_encoder_ticks / (wheel_diameter * 3.1416);


    public void NAV_init() {
        leftWheel = hardwareMap.dcMotor.get("left");
        rightWheel = hardwareMap.dcMotor.get("right");

        rightWheel.setDirection(DcMotorSimple.Direction.REVERSE);
        leftWheel.setDirection(DcMotorSimple.Direction.FORWARD);

        BNO055IMU.Parameters IMUParameters = new BNO055IMU.Parameters();
        IMUParameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        IMUParameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        IMUParameters.calibrationDataFile = "BNO055IMUCalibration.json";

        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(IMUParameters);

        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        while (!isStarted()) {
            telemetry.update();
            idle();
        }
    }

    void turn_to_heading(double target_heading) {
        boolean go_right;
        double current_heading;
        double degrees_to_turn;
        double wheel_power;

        DbgLog.msg("10435 starting turn_to_heading");
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        current_heading = angles.firstAngle;
        degrees_to_turn = Math.abs(target_heading - current_heading);

        go_right = target_heading > current_heading;

        if (degrees_to_turn > 180) {
            go_right = !go_right;
            degrees_to_turn = 360 - degrees_to_turn;
        }
        while (degrees_to_turn > .5 && opModeIsActive()) {

            wheel_power = (10 * Math.pow((degrees_to_turn + 13) / 40, 3) + 12) / 100;

            if (go_right) {
                wheel_power = -wheel_power;
            }

            rightWheel.setPower(wheel_power);
            leftWheel.setPower(-wheel_power);

            angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

            current_heading = angles.firstAngle;                               // get the new current reading
            degrees_to_turn = Math.abs(target_heading - current_heading);       // Calculate how far is remaining to turn

            go_right = target_heading > current_heading;

            if (degrees_to_turn > 180) {
                go_right = !go_right;
                degrees_to_turn = 360 - degrees_to_turn;
            }

        }

        leftWheel.setPower(0);
        rightWheel.setPower(0);
        sleep(300);

        DbgLog.msg("10435 ending turn_to_heading " + Double.toString(target_heading) + "  Attempted heading:" + Double.toString(current_heading) + "  Real current Heading:" + Double.toString(angles.firstAngle));

    } // end of turn_to_heading

    private double getSpeed() {
        double new_speed;

        speeds = imu.getVelocity();
        new_speed = speeds.xVeloc;

        return new_speed;
    }

    private double go_straight_adjustment(double target_heading) {

        //  This function outputs power_adjustment that should be added to right wheel and subtracted from left wheel

        double gs_adjustment;
        double current_heading;
        double degrees_off;
        boolean go_right;


        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        DbgLog.msg("10435 starting go_straight_adjustment heading:" + Double.toString(target_heading) + " current heading:" + Double.toString(angles.firstAngle));

        current_heading = angles.firstAngle;
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

        return gs_adjustment;

    } // end of go_straight_adjustment

    void go_forward(double inches_to_travel, int heading, double speed) {

        DbgLog.msg("10435 starting go_forward inches:" + Double.toString(inches_to_travel) + " heading:" + Integer.toString(heading) + " speed:" + Double.toString(speed) + " find white:");

        ElapsedTime log_timer = new ElapsedTime();

        double current_speed = .05;
        int ticks_to_travel;
        boolean destination_reached = false;
        boolean going_backwards = false;
        double speed_increase = .05;
        double actual_speed;
        int start_position_L;
        int start_position_R;
        int previous_ticks_traveled_L = 0;
        int ticks_traveled_L;
        int ticks_traveled_R;
        int lowest_ticks_traveled;
        double remaining_inches;
        double previous_log_timer = 0;
        double power_adjustment;

        if (speed < 0) {
            inches_to_travel = inches_to_travel * 1.08;
            speed_increase = -speed_increase;
            current_speed = -current_speed;
            going_backwards = true;
        }

        ticks_to_travel = (int) (inches_to_travel * ticks_per_inch);

        start_position_L = leftWheel.getCurrentPosition();
        start_position_R = rightWheel.getCurrentPosition();

        log_timer.reset();


        while (opModeIsActive() && !destination_reached) {

            telemetry.addData("go_forward ticks_to_travel", ticks_to_travel);
            telemetry.update();

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

            actual_speed = getSpeed();

            if (ticks_traveled_L != previous_ticks_traveled_L && log_timer.seconds() - previous_log_timer > .1) {
                DbgLog.msg("10435 go_forward ticks_traveled: L:" + Double.toString(ticks_traveled_L)
                        + " R:" + Double.toString(ticks_traveled_R) + " actual_speed:" + Double.toString(actual_speed));
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
                DbgLog.msg("10435 go_forward slowing down: remaining_inches:" + Double.toString(remaining_inches)
                        + " lowest_ticks_traveled:" + Integer.toString(lowest_ticks_traveled));
            }

        }

        rightWheel.setPower(0);
        leftWheel.setPower(0);

        //telemetry.addData("go_forward destination_reached", destination_reached);
        //telemetry.addData("go_forward touch_sensor_pressed", touch_sensor_pressed);
        //telemetry.addData("go_forward found_white", found_white);
        //telemetry.update();


        sleep(100);
        DbgLog.msg("10435 ending go_forward: opModeIsActive:" + Boolean.toString(opModeIsActive())
                + " distance traveled L:" + Double.toString((leftWheel.getCurrentPosition() - start_position_L) / ticks_per_inch)
                + " distance traveled R:" + Double.toString((rightWheel.getCurrentPosition() - start_position_R) / ticks_per_inch)
                + " destination_reached:" + Boolean.toString(destination_reached));

    } // end of go_forward

    String vuforia_scan() {

        ElapsedTime vuforiascantime = new ElapsedTime();

        boolean picturefound = false;

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);

        parameters.vuforiaLicenseKey = "AWaEPBn/////AAAAGWa1VK57tkUipP01PNk9ghlRuxjK1Oh1pmbHuRnpaJI0vi57dpbnIkpee7J1pQ2RIivfEFrobqblxS3dKUjRo52NMJab6Me2Yhz7ejs5SDn4G5dheW5enRNWmRBsL1n+9ica/nVjG8xvGc1bOBRsIeZyL3EZ2tKSJ407BRgMwNOmaLPBle1jxqAE+eLSoYsz/FuC1GD8c4S3luDm9Utsy/dM1W4dw0hDJFc+lve9tBKGBX0ggj6lpo9GUrTC8t19YJg58jsIXO/DiF09a5jlrTeB2LK+GndUDEGyZA1mS3yAR6aIBeDYnFw+79mVFIkTPk8wv3HIQfzoggCu0AwWJBVUVjkDxJOWfzCGjaHylZlo";

        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;
        this.vuforia = ClassFactory.createVuforiaLocalizer(parameters);

        VuforiaTrackables relicTrackables = this.vuforia.loadTrackablesFromAsset("RelicVuMark");
        VuforiaTrackable relicTemplate = relicTrackables.get(0);

        relicTrackables.activate();

        vuforiascantime.reset();
        while (opModeIsActive() && !picturefound && vuforiascantime.seconds() < 5) {
            RelicRecoveryVuMark vuMark = RelicRecoveryVuMark.from(relicTemplate);
            if (vuMark != RelicRecoveryVuMark.UNKNOWN) {
                telemetry.addData("VuMark", "%s visible", vuMark);
                picturereading = vuMark.toString();
                picturefound = true;
            } else {
                telemetry.addData("VuMark", "not visible");
                picturefound = false;
            }
            telemetry.update();
        }
            return picturereading;
    }

}


