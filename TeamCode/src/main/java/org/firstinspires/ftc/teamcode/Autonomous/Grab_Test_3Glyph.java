package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.GlobalVariables;

/**
 * Created by Drew on 1/21/2018.
 */
@Autonomous(name = "Grab Test 3Glyph", group = "Tests")
public class Grab_Test_3Glyph extends Mecanum_Nav_Routines {

    String[] vuforiareading = new String[2];

    @Override
    public void runOpMode() throws InterruptedException {
        MNav_Init();
        runtime.reset();

        vuforiareading[0] = "RIGHT";

        // turn on the wheel intake and turn around
        setwheelintake(true, true, false);
        turn_to_heading(210);

        leftclamp.setPosition(GlobalVariables.leftclampopenpos);
        rightclamp.setPosition(GlobalVariables.rightclampopenpos);

        // go to pile and grab
        goforwardstopdetect = .5;
        go_forward(23, 210, 1, false);
        wheelwaggle(160, true);
        go_forward(3, 210, 1, false);
        leftclamp.setPosition(GlobalVariables.leftclampclosedpos);
        rightclamp.setPosition(GlobalVariables.rightclampclosedpos);
        sleep(200);
        go_forward(9, 215, -1, false);
        leftclamp.setPosition(GlobalVariables.leftclampopenpos);
        rightclamp.setPosition(GlobalVariables.rightclampopenpos);
        go_forward(10.5, 210, .25, false);
        leftclamp.setPosition(GlobalVariables.leftclampclosedpos);
        rightclamp.setPosition(GlobalVariables.rightclampclosedpos);
        sleep(400);
        wheeljiggle();
        //leftclamp.setPosition(GlobalVariables.leftclampopenpos);
        //rightclamp.setPosition(GlobalVariables.rightclampopenpos);
        //go_forward(3, 210, .85, false);
        //leftclamp.setPosition(GlobalVariables.leftclampclosedpos);
        //rightclamp.setPosition(GlobalVariables.rightclampclosedpos);
        lift_glyph("up", 16, false);

        // back away from pile
        go_forward(15, 195, -.6, false);
/*
        turn_to_heading(0);
        setwheelintake(false, true, true);
        wall_distance_align(19);
        setwheelintake(false, false, true);

        // find the red line
        go_sideways("red", 90, 0, .6, 3, 0);

        if (vuforiareading[0] == "RIGHT") {
            go_sideways(null, 270, 0, .5, .38, 0);
        } else if (vuforiareading[0] == "LEFT") {
            go_sideways(null, 90, 0, .5, .455, 0);
        } else {
            go_sideways(null, 270, 0, .5, .38, 0);
        }

        // drop off the glyphs
        if (vuforiareading[0] != "CENTER") {
            lift_glyph("down", 3, false);
        }

        lift_glyph("down", 11, false);
        go_forward(8.5, 0, .5, false);

        if (vuforiareading[0] == "CENTER") {
            rightclamp.setPosition(GlobalVariables.rightclampinitpos);
            sleep(300);
            go_forward(4, 0, -.4, false);
            lift_glyph("down", 3, false);
            go_forward(4, 0, .4, false);
        }
        rightclamp.setPosition(GlobalVariables.rightclampinitpos);
        leftclamp.setPosition(GlobalVariables.leftclampinitpos);
        sleep(300);

        // back away from cryptobox
        go_forward(5, 0, -.2, true);
        lift_glyph("down", 0, false);
*/
    }
}
