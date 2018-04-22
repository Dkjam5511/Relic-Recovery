package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.GlobalVariables;

/**
 * Created by Drew on 1/21/2018.
 */
@Autonomous(name = "Bottom Red 3Glyph", group = "Autonomous")
public class Bottom_Red_3Glyph_Mecanum extends Mecanum_Nav_Routines {

    String[] vuforiareading = new String[2];

    @Override
    public void runOpMode() throws InterruptedException {
        MNav_Init();
        runtime.reset();
        rightclamp.setPosition(GlobalVariables.rightclampopenpos);  // Close the right clamp back to init so that intake arm drops
        leftclamp.setPosition(GlobalVariables.leftclampclosedpos);      // Close the left clamp onto the glyph
        vuforiareading[0] = jewelknockvuforia2("red", true);
        setwheelintake(false, false, true);
        lift_glyph("up", 7, false);
        rightclamp.setPosition(GlobalVariables.rightclampinitpos);
        go_forward(20, 0, .35, false);
        turn_to_heading(90);

        go_sideways("red", 270, 90, .35, 3, 16);

        if (vuforiareading[0] == "RIGHT") {
            go_sideways(null, 90, 90, .5, .93, 0);
        } else if (vuforiareading[0] == "LEFT") {
            go_sideways(null, 270, 90, .5, .35, 0);
        } else {
            go_sideways(null, 90, 90, .5, .35, 0);
        }

        // Placing the first glyph
        lift_glyph("down", 3, false);
        go_forward(7.5,90, .4, false);
        leftclamp.setPosition(GlobalVariables.leftclampinitpos);
        rightclamp.setPosition(GlobalVariables.rightclampinitpos);
        sleep(400);

        //Backing away from the cryptobox
        if (vuforiareading[0] == "RIGHT") {
            go_forward(9, 87, -1, false);
        } else if (vuforiareading[0] == "LEFT") {
            go_forward(9, 93, -1, false);
        } else {
            go_forward(9, 90, -1, false);
        }
        setwheelintake(true, true, false);
        turn_to_heading(270);

        if (vuforiareading[0] == "RIGHT") {
            go_sideways(null, 90, 270, .7, .55, 0);
        } else if (vuforiareading[0] == "LEFT") {
            go_sideways(null, 270, 270, .7, .4, 0);
        }

        // go to pile and grab
        goforwardstopdetect = .5;
        go_forward(12, 270, .7, false);
        wheelwaggle(160, false);
        go_forward(3, 270, 1, false);
        leftclamp.setPosition(GlobalVariables.leftclampclosedpos);
        rightclamp.setPosition(GlobalVariables.rightclampclosedpos);
        go_forward(9, 270, -1, false);
        leftclamp.setPosition(GlobalVariables.leftclampopenpos);
        rightclamp.setPosition(GlobalVariables.rightclampopenpos);
        go_forward(10.5, 270, .25, false);
        leftclamp.setPosition(GlobalVariables.leftclampclosedpos);
        rightclamp.setPosition(GlobalVariables.rightclampclosedpos);
        sleep(800);
        wheeljiggle();
        setwheelintake(false, false,true);
        lift_glyph("up", 16, false);

        // Back away from the pile
        go_forward(5, 290, -.5, false);
        setwheelintake(false, true, true);
        turn_to_heading(90);
        setwheelintake(false, false, true);
        wall_distance_align(15);

        go_sideways("red", 270, 90, .5, 3, 0);

        if (vuforiareading[0] == "LEFT") {
            go_sideways(null, 90, 90, .5, .3, 0);
        } else if (vuforiareading[0] == "RIGHT") {
            go_sideways(null, 270, 90, .5, .455, 0);
        } else {
            go_sideways(null, 90, 90, .5, .3, 0);
        }
        // drop off the glyphs
        if (vuforiareading[0] != "CENTER") {
            lift_glyph("down", 3, false);
        }

        go_forward(13, 90, .5, false);

        if (vuforiareading[0] == "CENTER") {
            leftclamp.setPosition(GlobalVariables.leftclampinitpos);
            sleep(700);
            go_forward(4, 90, -.4, false);
            lift_glyph("down", 3, false);
            go_forward(4, 90, .4, false);
        }
        leftclamp.setPosition(GlobalVariables.leftclampinitpos);
        rightclamp.setPosition(GlobalVariables.rightclampinitpos);
        sleep(400);
        go_forward(5, 90, -.2, true);
        lift_glyph("down", 0, false);
    }
}
