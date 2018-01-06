package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.GlobalVarriables;

/**
 * Created by Drew on 10/25/2017.
 */
@Autonomous(name = "Bottom_Blue", group = "Autonomous")
public class Bottom_Blue extends Navigation_Routines {

    String[] vuforiareading = new String[2];

    @Override
    public void runOpMode() throws InterruptedException {
        NAV_init();
        runtime.reset();
        lift_glyph("up");
        vuforiareading = vuforia_scan();
        jewelknockvuforia("blue", vuforiareading[1], true);

        if (vuforiareading[0] == "LEFT") {
            go_forward(24, 0, -.2, false);
            turn_to_heading_pirouette(50 , true);
            go_forward(6, 50, .2, false);
            sleep(300);
            lift_glyph("down");
            sleep(500);
            leftclamp.setPosition(GlobalVarriables.leftclampopen);
            rightclamp.setPosition(GlobalVarriables.rightclampopen);
            sleep(1000);
            go_forward(2, 50, -.2, true);
        } else if (vuforiareading[0] == "RIGHT") {
            go_forward(32.5, 0, -.2, false);
            turn_to_heading_pirouette(65, true);
            go_forward(6, 65, .2, false);
            sleep(300);
            lift_glyph("down");
            sleep(500);
            leftclamp.setPosition(GlobalVarriables.leftclampopen);
            rightclamp.setPosition(GlobalVarriables.rightclampopen);
            sleep(1000);
            go_forward(2, 65, -.2, true);
        } else {  //center
            go_forward(31, 0, -.2, false);
            turn_to_heading_pirouette(50, true);
            go_forward(6, 50, .2, false);
            sleep(300);
            lift_glyph("down");
            sleep(500);
            leftclamp.setPosition(GlobalVarriables.leftclampopen);
            rightclamp.setPosition(GlobalVarriables.rightclampopen);
            sleep(1000);
            go_forward(2, 50, -.2, true);
        }
    }
}