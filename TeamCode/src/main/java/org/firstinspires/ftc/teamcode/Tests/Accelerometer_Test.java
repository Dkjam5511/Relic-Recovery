package org.firstinspires.ftc.teamcode.Tests;


import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;


import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;
import org.firstinspires.ftc.teamcode.Autonomous.Mecanum_Nav_Routines;

/**
 * Created by Drew on 1/15/2018.
 */
@Autonomous(name = "Accelerometer Test", group = "Tests")
public class Accelerometer_Test extends Mecanum_Nav_Routines {

    BNO055IMU imu;
    Acceleration gravity;

    @Override
    public void runOpMode() throws InterruptedException {
        BNO055IMU.Parameters IMUParameters = new BNO055IMU.Parameters();
        IMUParameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        IMUParameters.calibrationDataFile = "BNO055IMUCalibration.json";

        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(IMUParameters);
        imu.startAccelerationIntegration(new Position(), new Velocity(), 1000);
        waitForStart();

        while(opModeIsActive()) {
            gravity  = imu.getGravity();
            telemetry.addData("Y Grav", gravity.yAccel);
            telemetry.update();
        }

    }
}
