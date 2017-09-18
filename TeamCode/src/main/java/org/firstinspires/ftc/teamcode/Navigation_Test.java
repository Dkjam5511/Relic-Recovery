package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

/**
 * Created by Drew on 9/17/2017.
 */

@Autonomous(name = "REV Navigation Test", group = "Test")
public class Navigation_Test extends Navigation_Routines {

    public void runOpMode() throws InterruptedException {

        NAV_init();

        go_forward(120,0,1);
        //turn_to_heading(90);
        //go_forward(24,90,.2);
        //turn_to_heading(0);
    }

}
