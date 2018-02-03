package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.GlobalVarriables;

/**
 * Created by Drew on 1/21/2018.
 */
@Autonomous(name = "Top Blue Mecanum", group = "Autonomous")
public class Top_Blue_Mecanum extends Mecanum_Nav_Routines {

    String[] vuforiareading = new String[2];

    @Override
    public void runOpMode() throws InterruptedException {
        MNav_Init();
        runtime.reset();
        lift_glyph("up", 5, true);
        vuforiareading = vuforia_scan();
        jewelknockvuforia("blue", vuforiareading[1], false);

        go_forward(22, 0, .12, false);
        lift_glyph("up", 15, false);
        go_sideways(null, 270, 0, .27, 2, 0);
        sleep(500);
        go_sideways("blue", 90, 0, .27, 10, 16);

        if (vuforiareading[0] == "LEFT") {
            go_sideways(null, 270, 0, .27, 1.5, 0);
        } else if (vuforiareading[0] == "RIGHT") {
            go_sideways(null, 90, 0, .27, 1.6, 0);
        } else {
            go_sideways(null, 90, 0, .27, .2, 0);
        }
        lift_glyph("down", 3,false);
        go_forward(7, 0, .15, false);
        sleep(300);
        lift_glyph("down", 0, false);
        sleep(500);
        leftclamp.setPosition(GlobalVarriables.leftclampopen);
        rightclamp.setPosition(GlobalVarriables.rightclampopen);
        sleep(1000);
        go_forward(4, 0, -.2, true);
    }
}
