package org.firstinspires.ftc.teamcode.Tests;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Autonomous.Mecanum_Nav_Routines;

/**
 * Created by Drew on 1/10/2018.
 */
@Autonomous (name = "Mecanum Autonous Test", group = "Tests")
public class Mecanum_Autonomus_Test extends Mecanum_Nav_Routines{

    //String[] vuforiareading = new String[2];

    @Override
    public void runOpMode() throws InterruptedException {
        MNav_Init();
        runtime.reset();
        lift_glyph("up", 6, true);
        go_sideways("red",90, 0,.27,20, 16);
        go_sideways(null,90, 0,.27,1.7, 0);
        go_forward(7,0,.15,false);
        //go_sideways(270,0,.3,5,0);
    }
}
