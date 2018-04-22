package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.GlobalVariables;

/**
 * Created by Drew on 1/21/2018.
 */
@Autonomous(name = "Top Blue 3Glyph", group = "Autonomous")
public class Top_Blue_3Glyph_Mecanum extends Mecanum_Nav_Routines {

    String[] vuforiareading = new String[2];

    @Override
    public void runOpMode() throws InterruptedException {
        MNav_Init();
        runtime.reset();
        rightclamp.setPosition(GlobalVariables.rightclampclosedpos);  // Close the both clamps so that we grab glyph and arrow falls
        leftclamp.setPosition(GlobalVariables.leftclampclosedpos);

        vuforiareading[0] = jewelknockvuforia2("blue", false);  // read the VuMark, recognize jewel color from image and knock off jewel
        if (vuforiareading[0] == "UNKNOWN") {
            vuforiareading[0] = "LEFT";
        }
        leftclamp.setPosition(GlobalVariables.leftclampinitpos);  // Open the unused clamp so it's out of the way

        // Deploy intake wheels and lift the glyph
        setwheelintake(false, true, true);
        lift_glyph("up", 7, false);

        // Go off the platform
        go_forward(22, 0, .3, false);
        setwheelintake(false, false, true);
        sleep(100);

        // find the blue line
        go_sideways("blue", 90, 0, .4, 1.5, 0);

        // move to the correct cryptobox column
        if (vuforiareading[0] == "LEFT") {
            go_sideways(null, 270, 0, .5, .825, 0);
        } else if (vuforiareading[0] == "RIGHT") {
            go_sideways(null, 90, 0, .5, .37, 0);
        } else {
            go_sideways(null, 270, 0, .5, .4, 0);
        }

        //  Place first glyph in cryptobox
        lift_glyph("down", 3, false);
        go_forward(7, 0, .4, false);
        leftclamp.setPosition(GlobalVariables.leftclampopenpos);
        rightclamp.setPosition(GlobalVariables.rightclampopenpos);
        sleep(400);

        //  Back away from cryptobox
        go_forward(7.5, 0, -.6, false);

        // Move to the right to get around the platform
        if (vuforiareading[0] == "LEFT") {
            go_sideways(null, 90, 0, .5, 1.3, 0);
        } else if (vuforiareading[0] == "RIGHT") {
            go_sideways(null, 90, 0, .5, .45, 0);
        } else {
            go_sideways(null, 90, 0, .5, .875, 0);
        }

        // turn on the wheel intake and turn around
        setwheelintake(true, true, false);
        turn_to_heading(155);

        // go to pile and grab
        goforwardstopdetect = .5;
        go_forward(23, 150, 1, false);
        wheelwaggle(160, true);
        go_forward(5, 150, 1, false);
        leftclamp.setPosition(GlobalVariables.leftclampclosedpos);
        rightclamp.setPosition(GlobalVariables.rightclampclosedpos);
        sleep(200);
        go_forward(9, 145, -1, false);
        leftclamp.setPosition(GlobalVariables.leftclampopenpos);
        rightclamp.setPosition(GlobalVariables.rightclampopenpos);
        go_forward(13.5, 150, .25, false);
        goforwardstopdetect = 2;
        leftclamp.setPosition(GlobalVariables.leftclampclosedpos);
        rightclamp.setPosition(GlobalVariables.rightclampclosedpos);
        sleep(400);
        wheeljiggle();
        setwheelintake(false, true, true);
        lift_glyph("up", 16, false);
        setwheelintake(true, true, false);

        // back away from pile
        go_forward(15, 170, -.6, false);

        // get straight with cryptobox
        turn_to_heading(260);
        turn_to_heading(0);
        setwheelintake(false, true, true);
        wall_distance_align(19);
        setwheelintake(false, false, true);

        // find the blue line
        go_sideways("blue", 270, 0, .5, 3, 0);

        if (vuforiareading[0] == "LEFT") {
            go_sideways(null, 90, 0, .5, .3, 0);
        } else if (vuforiareading[0] == "RIGHT") {
            go_sideways(null, 270, 0, .5, .455, 0);
        } else {
            go_sideways(null, 90, 0, .5, .3, 0);
        }
/*
        // go sideways to correct column
        if (vuforiareading[0] == "LEFT") {
            go_sideways(null, 270, 0, .5, .5, 0);
        } else if (vuforiareading[0] == "RIGHT") {
            go_sideways(null, 270, 0, .5, .95, 0);
        } else {
            go_sideways(null, 270, 0, .5, .5, 0);
        }
*/
        // drop off the glyphs
        if (vuforiareading[0] == "CENTER") {
            lift_glyph("down", 11, false);
        } else {
            lift_glyph("down", 3, false);
        }

        go_forward(8.5, 0, .5, false);

        if (vuforiareading[0] == "CENTER") {
            leftclamp.setPosition(GlobalVariables.leftclampinitpos);
            sleep(600);
            go_forward(4, 0, -.4, false);
            lift_glyph("down", 3, false);
            go_forward(4, 0, .4, false);
        }
        leftclamp.setPosition(GlobalVariables.leftclampinitpos);
        rightclamp.setPosition(GlobalVariables.rightclampinitpos);
        sleep(300);

        // back away from cryptobox
        go_forward(5, 0, -.2, true);
        lift_glyph("down", 0, false);

    }
}
