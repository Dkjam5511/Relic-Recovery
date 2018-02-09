package org.firstinspires.ftc.teamcode.Tests;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Autonomous.Mecanum_Nav_Routines;

/**
 * Created by Drew on 1/10/2018.
 */
@Autonomous(name = "Mecanum Autonous Test", group = "Tests")
public class Mecanum_Autonomus_Test extends Mecanum_Nav_Routines {

    @Override
    public void runOpMode() throws InterruptedException {
        MNav_Init();
        runtime.reset();
        lift_glyph("up", 15, false);
        go_sideways(null, 90, 0, .35, 2, 16);
        sleep(1000);
        go_sideways(null, 90, 0, .35, 1.5, 16);
        sleep(1000);
        go_sideways(null, 90, 0, .35, 1.7, 16);
        sleep(1000);
        go_sideways(null, 90, 0, .35, .2, 16);
        sleep(1000);
        go_sideways(null, 90, 0, .35, 10, 16);
    }

}

