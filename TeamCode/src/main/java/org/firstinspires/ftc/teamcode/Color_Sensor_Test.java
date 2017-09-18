package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;

/**
 * Created by Drew on 9/16/2017.
 */
@TeleOp(name = "CS_Test", group = "Tests")
public class Color_Sensor_Test extends OpMode {

    ColorSensor color_sensor;

    @Override
    public void init() {
        color_sensor = hardwareMap.colorSensor.get("cs");
        color_sensor.enableLed(false);
    }

    @Override
    public void loop() {
        telemetry.addData("Red :", color_sensor.red());
        telemetry.addData("Blue :", color_sensor.blue());
        telemetry.addData("Green :", color_sensor.green());
        telemetry.addData("Alpha :", color_sensor.alpha());
    }
}
