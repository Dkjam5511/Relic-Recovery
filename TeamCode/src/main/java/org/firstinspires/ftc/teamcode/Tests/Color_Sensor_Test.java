package org.firstinspires.ftc.teamcode.Tests;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Navigation_Routines;

/**
 * Created by Drew on 9/16/2017.
 */
@Autonomous(name = "CS_Test", group = "Tests")
public class Color_Sensor_Test extends Navigation_Routines {

    @Override
    public void runOpMode() throws InterruptedException {
        NAV_init();
        jewelknockside("red");
        sleep(10000);
    }


}
