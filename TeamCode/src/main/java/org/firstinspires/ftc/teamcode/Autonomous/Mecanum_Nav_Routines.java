package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.DbgLog;

/**
 * Created by Drew on 1/10/2018.
 */

abstract public class Mecanum_Nav_Routines extends LinearOpMode{

    DcMotor leftFront;
    DcMotor rightFront;
    DcMotor leftRear;
    DcMotor rightRear;
    BNO055IMU imu;
    Orientation angles;

    double gs_previous_speed;
    double gs_previous_ticks_traveled;
    boolean gs_first_run = true;
    ElapsedTime gs_speed_timer = new ElapsedTime();

    private double wheel_encoder_ticks = 537.6;
    private double wheel_diameter = 3.75;  // size of wheels
    public double ticks_per_inch = wheel_encoder_ticks / (wheel_diameter * 3.1416);

    public ElapsedTime runtime = new ElapsedTime();

    public void MNav_Init(){
        leftFront = hardwareMap.dcMotor.get("lf");
        rightFront = hardwareMap.dcMotor.get("rf");
        leftRear = hardwareMap.dcMotor.get("lr");
        rightRear = hardwareMap.dcMotor.get("rr");

        rightFront.setDirection(DcMotor.Direction.REVERSE);
        rightRear.setDirection(DcMotor.Direction.REVERSE);

        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        BNO055IMU.Parameters IMUParameters = new BNO055IMU.Parameters();
        IMUParameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        IMUParameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        IMUParameters.calibrationDataFile = "BNO055IMUCalibration.json";

        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(IMUParameters);

        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

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

            wheel_power = (2 * Math.pow((degrees_to_turn + 13) / 23, 2) + 10) / 100;

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
                    leftRear.setPower(-wheel_power*.8);
                    leftFront.setPower(-wheel_power*1.4);
                    rightFront.setPower(wheel_power);
                    rightRear.setPower(0);
                } else {
                    rightRear.setPower(wheel_power*.8);
                    rightFront.setPower(wheel_power*1.4);
                    leftFront.setPower(-wheel_power);
                    leftRear.setPower(0);
                }
            } else {
                if (go_backwards) {
                    rightRear.setPower(-wheel_power*.8);
                    rightFront.setPower(-wheel_power*1.4);
                    leftFront.setPower(wheel_power);
                    leftRear.setPower(0);
                } else {
                    leftRear.setPower(wheel_power*.8);
                    leftFront.setPower(wheel_power*1.4);
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
        double lagreduction = 1.125;
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
}

