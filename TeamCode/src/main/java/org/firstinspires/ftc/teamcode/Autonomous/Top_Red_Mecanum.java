package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.GlobalVarriables;

/**
 * Created by Drew on 1/21/2018.
 */
@Autonomous(name = "Top Red", group = "Autonomous")
public class Top_Red_Mecanum extends Mecanum_Nav_Routines {

    String[] vuforiareading = new String[2];

    @Override
    public void runOpMode() throws InterruptedException {
        MNav_Init();
        runtime.reset();
        setwheelintake(false, true, true);
        leftclamp.setPosition(GlobalVarriables.leftclampdeploypos);
        rightclamp.setPosition(GlobalVarriables.rightclampdeploypos);
        sleep(500);
        rightclamp.setPosition(GlobalVarriables.rightclampopenpos);  // Close the right clamp on the glyph
        leftclamp.setPosition(GlobalVarriables.leftclampclosedpos);      // put the left clamp back to init so that intake arm drops
        vuforiareading[0] = jewelknockvuforia2("red", true);
        setwheelintake(false, false, true);
        lift_glyph("up", 7, false);
        rightclamp.setPosition(GlobalVarriables.rightclampinitpos);

        go_forward(24, 0, .12, false);
        lift_glyph("up", 15, false);
        go_sideways(null, 90, 0, .27, 2, 0);
        sleep(500);
        go_sideways("red", 270, 0, .27, 10, 16.5);

        if (vuforiareading[0] == "LEFT") {
            go_sideways(null, 270, 0, .27, 2, 0);
        } else if (vuforiareading[0] == "RIGHT") {
            go_sideways(null, 90, 0, .27, 2, 0);
        } else { //center
            go_sideways(null, 270, 0, .27, .2, 0);
        }
        lift_glyph("down", 3,false);
        go_forward(7, 0, .15, false);
        sleep(300);
        lift_glyph("down", 0, false);
        sleep(500);
        leftclamp.setPosition(GlobalVarriables.leftclampinitpos);
        rightclamp.setPosition(GlobalVarriables.rightclampinitpos);
        sleep(1000);
        go_forward(4, 0, -.2, true);//center
    }
}
