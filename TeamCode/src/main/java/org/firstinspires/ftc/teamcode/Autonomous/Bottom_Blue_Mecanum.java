package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.GlobalVariables;

/**
 * Created by Drew on 1/21/2018.
 */
@Autonomous(name = "Bottom Blue", group = "Autonomous")
public class Bottom_Blue_Mecanum extends Mecanum_Nav_Routines {

    String[] vuforiareading = new String[2];

    @Override
    public void runOpMode() throws InterruptedException {
        MNav_Init();
        runtime.reset();
        leftclamp.setPosition(GlobalVariables.leftclampinitpos);
        rightclamp.setPosition(GlobalVariables.rightclampinitpos);
        sleep(500);
        setwheelintake(false, true, true);
        rightclamp.setPosition(GlobalVariables.rightclampclosedpos);  // Close the right clamp on the glyph
        leftclamp.setPosition(GlobalVariables.leftclampopenpos);      // put the left clamp back to init so that intake arm drops
        vuforiareading[0] = jewelknockvuforia2("blue", false);
        setwheelintake(false, false, true);
        lift_glyph("up", 7, false);
        leftclamp.setPosition(GlobalVariables.leftclampinitpos);
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
        leftclamp.setPosition(GlobalVariables.leftclampopenpos);
        rightclamp.setPosition(GlobalVariables.rightclampopenpos);
        sleep(400);
        go_forward(5, 270, -.2, true);
        lift_glyph("down", 0, false);
    }
}