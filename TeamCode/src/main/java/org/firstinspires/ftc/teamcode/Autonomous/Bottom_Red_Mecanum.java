package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.GlobalVariables;

/**
 * Created by Drew on 1/21/2018.
 */
@Autonomous(name = "Bottom Red", group = "Autonomous")
public class Bottom_Red_Mecanum extends Mecanum_Nav_Routines {

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
        go_forward(7.5, 90, .4, false);
        leftclamp.setPosition(GlobalVariables.leftclampinitpos);
        rightclamp.setPosition(GlobalVariables.rightclampinitpos);
        sleep(400);
        go_forward(5, 90, -.2, true);
        lift_glyph("down", 0, false);
    }
}