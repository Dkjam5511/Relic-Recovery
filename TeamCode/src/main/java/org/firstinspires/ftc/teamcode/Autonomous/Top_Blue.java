package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.GlobalVarriables;

/**
 * Created by Drew on 10/25/2017.
 */
@Autonomous(name = "Top_Blue", group = "Autonomous")
public class Top_Blue extends Navigation_Routines {

    String[] vuforiareading = new String[2];

    @Override
    public void runOpMode() throws InterruptedException {
        NAV_init();
        runtime.reset();
        lift_glyph("up");
        vuforiareading = vuforia_scan();
        jewelknockvuforia("blue", vuforiareading[1], false);

        if (vuforiareading[0] == "LEFT") {
            go_forward(20, 0, .2, false);
            turn_to_heading_pirouette(232 , false);
            go_forward(16, 232, .2, false);
            sleep(300);
            lift_glyph("down");
            sleep(500);
            leftclamp.setPosition(GlobalVarriables.leftclampopen);
            rightclamp.setPosition(GlobalVarriables.rightclampopen);
            sleep(1000);
            go_forward(2, 232, -.2, true);
        } else if (vuforiareading[0] == "RIGHT") {
            go_forward(26, 0, .2, false);
            turn_to_heading_pirouette(270, false);
            go_forward(11, 270, .2, false);
            turn_to_heading_pirouette(240, false);
            go_forward(5.5, 240, .2, false);
            sleep(300);
            lift_glyph("down");
            sleep(500);
            leftclamp.setPosition(GlobalVarriables.leftclampopen);
            rightclamp.setPosition(GlobalVarriables.rightclampopen);
            sleep(1000);
            go_forward(2, 240, -.2, true);

        } else {  //center
            go_forward(26, 0, .2, false);
            turn_to_heading_pirouette(236, false);
            go_forward(15.5, 236, .2, false);
            sleep(300);
            lift_glyph("down");
            sleep(500);
            leftclamp.setPosition(GlobalVarriables.leftclampopen);
            rightclamp.setPosition(GlobalVarriables.rightclampopen);
            sleep(1000);
            go_forward(2, 236, -.2, true);
        }
    }
}