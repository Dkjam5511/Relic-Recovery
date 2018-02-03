package org.firstinspires.ftc.teamcode.Tests;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;

/**
 * Created by Drew on 1/29/2018.
 */
@Autonomous (name = "Color Sensor Test", group = "Tests")
public class Color_Sensor_Test extends LinearOpMode {

    ColorSensor cs;

    @Override
    public void runOpMode() throws InterruptedException {

        cs = hardwareMap.colorSensor.get("cs");

        waitForStart();

        while (opModeIsActive()) {
            telemetry.addData("Red", cs.red());
            telemetry.addData("Blue", cs.blue());
            telemetry.update();
        }
    }
}
