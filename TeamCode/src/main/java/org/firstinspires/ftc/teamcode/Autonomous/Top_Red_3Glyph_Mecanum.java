package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.GlobalVariables;

/**
 * Created by Drew on 1/21/2018.
 */
@Autonomous(name = "Top Red 3Glyph", group = "Autonomous")
public class Top_Red_3Glyph_Mecanum extends Mecanum_Nav_Routines {

    String[] vuforiareading = new String[2];
    int degrees_away_from_platform = 25;

    @Override
    public void runOpMode() throws InterruptedException {
        MNav_Init();
        runtime.reset();
        rightclamp.setPosition(GlobalVariables.rightclampclosedpos);  // Close the both clamps so that we grab glyph and arrow falls
        leftclamp.setPosition(GlobalVariables.leftclampclosedpos);

        vuforiareading[0] = jewelknockvuforia2("red", true);  // read the VuMark, recognize jewel color from image and knock off jewel
        if (vuforiareading[0] == "UNKNOWN") {
            vuforiareading[0] = "RIGHT";
        }
        rightclamp.setPosition(GlobalVariables.rightclampinitpos);  // Open the unused clamp so it's out of the way

        // Deploy intake wheels and lift the glyph
        setwheelintake(false, true, true);
        lift_glyph("up", 7, false);

        // Go off the platform
        go_forward(22.5, 0, .3, false);
        setwheelintake(false, false, true);
        sleep(100);

        // find the red line
        go_sideways("red", 270, 0, .4, 1.5, 0);

        // move to the correct cryptobox column
        if (vuforiareading[0] == "RIGHT") {
            go_sideways(null, 90, 0, .5, .93, 0);
        } else if (vuforiareading[0] == "LEFT") {
            go_sideways(null, 270, 0, .5, .35, 0);
        } else {
            go_sideways(null, 90, 0, .5, .35, 0);
        }

        //  Place first glyph in cryptobox
        lift_glyph("down", 3, false);
        go_forward(7, 0, .4, false);
        leftclamp.setPosition(GlobalVariables.leftclampinitpos);
        rightclamp.setPosition(GlobalVariables.rightclampinitpos);
        sleep(400);

        //  Back away from cryptobox
        go_forward(7, 0, -.6, false);

        // Move to the left to get around the platform
        if (vuforiareading[0] == "RIGHT") {
            go_sideways(null, 270, 0, .5, 1.3, 0);
        } else if (vuforiareading[0] == "LEFT") {
            go_sideways(null, 270, 0, .5, .45, 0);
        } else {
            go_sideways(null, 270, 0, .5, .875, 0);
        }

        // turn on the wheel intake and turn around
        setwheelintake(true, true, false);
        turn_to_heading(180 + degrees_away_from_platform);

        leftclamp.setPosition(GlobalVariables.leftclampopenpos);
        rightclamp.setPosition(GlobalVariables.rightclampopenpos);

        // go to pile and grab
        goforwardstopdetect = .5;
        go_forward(24, 180 + degrees_away_from_platform, .7, false);
        wheelwaggle(160, false);
        go_forward(3.5, 180 + degrees_away_from_platform, 1, false);
        leftclamp.setPosition(GlobalVariables.leftclampclosedpos);
        rightclamp.setPosition(GlobalVariables.rightclampclosedpos);
        sleep(200);
        go_forward(9, 180 + degrees_away_from_platform, -1, false);
        leftclamp.setPosition(GlobalVariables.leftclampopenpos);
        rightclamp.setPosition(GlobalVariables.rightclampopenpos);
        go_forward(10.5, 180 + degrees_away_from_platform, .25, false);
        leftclamp.setPosition(GlobalVariables.leftclampclosedpos);
        rightclamp.setPosition(GlobalVariables.rightclampclosedpos);
        sleep(300);
        wheeljiggle();
        //leftclamp.setPosition(GlobalVariables.leftclampopenpos);
        //rightclamp.setPosition(GlobalVariables.rightclampopenpos);
        //go_forward(3, 210, .85, false);
        //leftclamp.setPosition(GlobalVariables.leftclampclosedpos);
        //rightclamp.setPosition(GlobalVariables.rightclampclosedpos);
        lift_glyph("up", 16, false);

        // back away from pile
        go_forward(15, 195, -.6, false);

        turn_to_heading(0);
        setwheelintake(false, true, true);
        wall_distance_align(19);
        setwheelintake(false, false, true);

        // find the red line
        go_sideways("red", 90, 0, .6, 3, 0);

        // move to the correct cryptobox column
        if (vuforiareading[0] == "RIGHT") {
            go_sideways(null, 270, 0, .5, .38, 0);
        } else if (vuforiareading[0] == "LEFT") {
            go_sideways(null, 90, 0, .5, .42, 0);
        } else {
            go_sideways(null, 270, 0, .5, .38, 0);
        }

        // drop off the glyphs
        if (vuforiareading[0] == "CENTER") {
            lift_glyph("down", 11, false);
        } else {
            lift_glyph("down", 3, false);
        }

        go_forward(8.5, 0, .5, false);

        if (vuforiareading[0] == "CENTER") {
            rightclamp.setPosition(GlobalVariables.rightclampinitpos);
            sleep(600);
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

    }
}
