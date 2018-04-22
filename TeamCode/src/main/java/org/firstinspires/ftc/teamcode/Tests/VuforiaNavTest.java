package org.firstinspires.ftc.teamcode.Tests;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Autonomous.Old_Autonomous.Nav_Routines;

/**
 * Created by Drew on 10/2/2017.
 */
@Autonomous(name = "VuforiaNavTest", group = "Tests")
public class VuforiaNavTest extends Nav_Routines {
    @Override
    public void runOpMode() throws InterruptedException {
        NAV_init();
        vuforia_scan10435();
    }
}
