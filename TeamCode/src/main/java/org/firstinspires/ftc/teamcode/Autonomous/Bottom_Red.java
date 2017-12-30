package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Navigation_Routines;
import org.firstinspires.ftc.teamcode.GlobalVarriables;

/**
 * Created by Drew on 10/25/2017.
 */
@Autonomous(name = "Bottom_Red", group = "Autonomous")
public class Bottom_Red extends Navigation_Routines {

    String vuforiareading;

    @Override
    public void runOpMode() throws InterruptedException {
        NAV_init();
        runtime.reset();
        lift_glyph("up");
        vuforiareading = vuforia_scan();
        jewelknockside("red");

        if (vuforiareading == "LEFT") {
            go_forward(42, 0, .2, false);
            turn_to_heading_pirouette(50, true);
            go_forward(7, 50, .2, false);
            sleep(300);
            lift_glyph("down");
            sleep(500);
            leftclamp.setPosition(GlobalVarriables.leftclampopen);
            rightclamp.setPosition(GlobalVarriables.rightclampopen);
            sleep(1000);
            go_forward(2, 50, -.2, true);
        } else if (vuforiareading == "RIGHT") {
            go_forward(34, 0, .2, false);
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
            go_forward(35.5, 0, .2, false);
            turn_to_heading_pirouette(52, true);
            go_forward(6, 52, .2, false);
            sleep(300);
            lift_glyph("down");
            sleep(500);
            leftclamp.setPosition(GlobalVarriables.leftclampopen);
            rightclamp.setPosition(GlobalVarriables.rightclampopen);
            sleep(1000);
            go_forward(2, 52, -.2, true);
        }
    }
}