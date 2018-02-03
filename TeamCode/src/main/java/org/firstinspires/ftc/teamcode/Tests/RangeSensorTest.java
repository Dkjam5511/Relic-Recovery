package org.firstinspires.ftc.teamcode.Tests;


import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;


import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Autonomous.Mecanum_Nav_Routines;

/**
 * Created by Drew on 1/15/2018.
 */
@Autonomous(name = "Range Sensor Test", group = "Tests")
public class RangeSensorTest extends Mecanum_Nav_Routines {

    ModernRoboticsI2cRangeSensor rangesensor;

    @Override
    public void runOpMode() throws InterruptedException {
        MNav_Init();
        runtime.reset();
        rangesensor = hardwareMap.get(ModernRoboticsI2cRangeSensor.class, "rs");
        waitForStart();
        lift_glyph("up", 15, true);
        while(opModeIsActive()) {
            telemetry.addData("Raw Ultrasonic", rangesensor.rawUltrasonic());
            telemetry.addData("Raw Optical", rangesensor.rawOptical());
            telemetry.addData("Distance", rangesensor.getDistance(DistanceUnit.INCH));
            telemetry.update();
        }

    }
}
