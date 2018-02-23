package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.GlobalVarriables;

/**
 * Created by Drew on 1/21/2018.
 */
@Autonomous(name = "Bottom Red 3Glyph Mecanum", group = "Autonomous")
public class Bottom_Red_3Glyph_Mecanum extends Mecanum_Nav_Routines {

    String[] vuforiareading = new String[2];

    @Override
    public void runOpMode() throws InterruptedException {
        MNav_Init();
        runtime.reset();
        lift_glyph("up", 15, true);
        vuforiareading = vuforia_scan();
        jewelknockvuforia("red", vuforiareading[1], true);

        go_forward(28, 0, .5, false);
        turn_to_heading(90);
        //lift_glyph("up", 15, false);
        //go_sideways(null, 90, 0, .27, 2.5, 0);
        //sleep(500);
        go_forward(2, 0, .2, false);

        go_sideways("red", 270, 90, .35, 3, 16);

        if (vuforiareading[0] == "LEFT") {
            go_sideways(null, 270, 90, .27, 1.7, 0);
        } else if (vuforiareading[0] == "RIGHT") {
            go_sideways(null, 90, 90, .27, 2.5, 0);
        } else {
            go_sideways(null, 270, 90, .27, .2, 0);
        }

        lift_glyph("down", 1,false);
        go_forward(6, 90, .4, false);
        //sleep(300);
        //lift_glyph("down", 0, false);
        //sleep(100);
        leftclamp.setPosition(GlobalVarriables.leftclampopenpos);
        rightclamp.setPosition(GlobalVarriables.rightclampopenpos);
        sleep(400);
        go_forward(8, 90, -.9, true);
        turn_to_heading(270);
        setwheelintake(true, true);
        go_forward(9, 270, .2, true);
        lift_glyph("up", 2, true);
        setwheelintake(false, false);
        go_forward(10, 270, -.5, true);
        turn_to_heading(90);
        lift_glyph("up", 12, true);
        go_forward(12, 90, .5, true);
        leftclamp.setPosition(GlobalVarriables.leftclampopenpos);
        rightclamp.setPosition(GlobalVarriables.rightclampopenpos);
        setwheelintake(false, true);
        sleep(400);
        go_forward(5, 90, -.2, true);
    }
}
