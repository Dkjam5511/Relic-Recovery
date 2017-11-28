package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Navigation_Routines;
import org.firstinspires.ftc.teamcode.GlobalVarriables;

/**
 * Created by Drew on 10/25/2017.
 */
@Autonomous (name = "Top_Blue", group = "Autonomous")
public class Top_Blue extends Navigation_Routines {

    String vuforiareading;

    @Override
    public void runOpMode() throws InterruptedException {
        NAV_init();
        runtime.reset();
        lift_glyph("up");
        vuforiareading = vuforia_scan();
        jewelknock("blue");

        if (vuforiareading == "LEFT"){
            go_forward(23, 0,.7);
            turn_to_heading(240);
            go_forward(26,240,.7);
            leftclamp.setPosition(GlobalVarriables.leftclampopen);
            rightclamp.setPosition(GlobalVarriables.rightclampopen);
            sleep(2000);
            lift_glyph("down");
            go_forward(2,240,-.7);
        } else if (vuforiareading == "RIGHT"){
            go_forward(27.5, 0,.7);
           turn_to_heading(270);
           go_forward(13.5,270,.7);
           turn_to_heading(240);
           go_forward(10,240, .7);
            leftclamp.setPosition(GlobalVarriables.leftclampopen);
            rightclamp.setPosition(GlobalVarriables.rightclampopen);
            sleep(2000);
            lift_glyph("down");
            go_forward(1,240,-.7);
        } else {
            go_forward(27, 0,.7);
            turn_to_heading(246);
            go_forward(26,246,.7);
            leftclamp.setPosition(GlobalVarriables.leftclampopen);
            rightclamp.setPosition(GlobalVarriables.rightclampopen);
            sleep(2000);
            lift_glyph("down");
            go_forward(1,240,-.7);
        }
    }
}