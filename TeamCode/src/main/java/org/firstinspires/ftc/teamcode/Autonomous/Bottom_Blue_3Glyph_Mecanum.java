package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.GlobalVarriables;

/**
 * Created by Drew on 1/21/2018.
 */
@Autonomous(name = "Bottom Blue 3Glyph", group = "Autonomous")
public class Bottom_Blue_3Glyph_Mecanum extends Mecanum_Nav_Routines {

    String[] vuforiareading = new String[2];

    @Override
    public void runOpMode() throws InterruptedException {
        MNav_Init();
        runtime.reset();
        leftclamp.setPosition(GlobalVarriables.leftclampdeploypos);
        rightclamp.setPosition(GlobalVarriables.rightclampdeploypos);
        sleep(500);
        setwheelintake(false, true, true);
        rightclamp.setPosition(GlobalVarriables.rightclampclosedpos);  // Close the right clamp on the glyph
        leftclamp.setPosition(GlobalVarriables.leftclampopenpos);      // put the left clamp back to init so that intake arm drops
        vuforiareading[0] = jewelknockvuforia2("blue", false);
        setwheelintake(false, false, true);
        lift_glyph("up", 7, false);
        leftclamp.setPosition(GlobalVarriables.leftclampinitpos);
        go_forward(20, 0, .35, false);
        turn_to_heading(270);
        go_forward(3, 270, .3, false);

        go_sideways("blue", 90, 270, .35, 3, 16);

        if (vuforiareading[0] == "RIGHT") {
            go_sideways(null, 90, 270, .35, .85, 0);
        } else if (vuforiareading[0] == "LEFT") {
            go_sideways(null, 270, 270, .35, 1.2, 0);
        } else {
            go_sideways(null, 270, 270, .35, .4, 0);
        }

        lift_glyph("down", 3, false);
        go_forward(7.5, 270, .4, false);
        leftclamp.setPosition(GlobalVarriables.leftclampopenpos);
        rightclamp.setPosition(GlobalVarriables.rightclampopenpos);
        sleep(400);

        if (vuforiareading[0] == "RIGHT") {
            go_forward(9, 267, -1, false);
        } else if (vuforiareading[0] == "LEFT") {
            go_forward(9, 273, -1, false);
        } else {
            go_forward(9, 270, -1, false);
        }
        setwheelintake(true, true, false);
        turn_to_heading(90);
        if (vuforiareading[0] == "RIGHT") {
            go_sideways(null, 90, 90, .7, .45, 0);
        } else if (vuforiareading[0] == "LEFT") {
            go_sideways(null, 270, 90, .7, .75, 0);
        }
        go_forward(9, 90, 1, false);
        leftclamp.setPosition(GlobalVarriables.leftclampclosedpos);
        rightclamp.setPosition(GlobalVarriables.rightclampclosedpos);
        go_forward(9, 90, -1, false);
        leftclamp.setPosition(GlobalVarriables.leftclampopenpos);
        rightclamp.setPosition(GlobalVarriables.rightclampopenpos);
        go_forward(9, 90, .25, false);
        leftclamp.setPosition(GlobalVarriables.leftclampclosedpos);
        rightclamp.setPosition(GlobalVarriables.rightclampclosedpos);
        sleep(800);
        wheeljiggle();
        setwheelintake(false, false,true);
        lift_glyph("up", 12, false);
        go_forward(6, 90, -.5, false);
        setwheelintake(false, true, true);
        turn_to_heading(270);
        setwheelintake(false, false, true);
        if (vuforiareading[0] != "CENTER") {
            lift_glyph("down", 3, false);
        }
        go_forward(10.5, 270, .5, false);

        if (vuforiareading[0] == "CENTER") {
            rightclamp.setPosition(GlobalVarriables.rightclampinitpos);
            sleep(400);
            go_forward(4, 270, -.4, false);
            lift_glyph("down", 3, false);
            go_forward(4, 270, .4, false);
        } else{
            rightclamp.setPosition(GlobalVarriables.rightclampinitpos);
        }
        leftclamp.setPosition(GlobalVarriables.leftclampinitpos);
        sleep(400);
        go_forward(5, 270, -.2, true);
        lift_glyph("down", 0, false);
    }
}
