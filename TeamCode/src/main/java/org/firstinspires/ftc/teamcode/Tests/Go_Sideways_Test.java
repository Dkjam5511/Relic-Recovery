package org.firstinspires.ftc.teamcode.Tests;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Autonomous.Mecanum_Nav_Routines;

@Autonomous (name = "Go Sideways Test", group = "Tests")
public class Go_Sideways_Test extends Mecanum_Nav_Routines {
    @Override
    public void runOpMode() throws InterruptedException {
        MNav_Init();
        runtime.reset();
        go_sideways(null, 270, 0, .5, 2, 0);
        go_sideways(null, 90, 0, .5, 2, 0);
        go_sideways(null, 270, 0, 1, 1.5, 0);
        go_sideways(null, 90, 0, 1, 1.5, 0);

        /*
        go_sideways(null, 270, 0, .5, 1.3, 0);
        sleep(500);
        go_sideways(null, 90,0,.5,.875,0);
        sleep(3000);
        go_sideways(null, 270, 0, .5, .875, 0);
        sleep(500);
        go_sideways(null, 90,0,.5,.45,0);
        sleep(3000);
        go_sideways(null, 270, 0, .5, .45, 0);
        */

    }
}
