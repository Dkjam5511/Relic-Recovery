package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Navigation_Routines;
import org.firstinspires.ftc.teamcode.GlobalVarriables;

/**
 * Created by Drew on 10/25/2017.
 */
@Autonomous (name = "Top_Red", group = "Autonomous")
public class Top_Red extends Navigation_Routines {

    String vuforiareading;

    @Override
    public void runOpMode() throws InterruptedException {
        NAV_init();
        runtime.reset();
        lift_glyph("up");
        vuforiareading = vuforia_scan();
        jewelknock("red");

        if (vuforiareading == "RIGHT"){
            go_forward(20, 0,.2, false);
            turn_to_heading_pirouette(128, false);
            go_forward(16,128,.2, false);
            sleep(300);
            lift_glyph("down");
            sleep(500);
            leftclamp.setPosition(GlobalVarriables.leftclampopen);
            rightclamp.setPosition(GlobalVarriables.rightclampopen);
            sleep(1000);
            go_forward(2,128,-.2, true);


        } else if (vuforiareading == "LEFT"){
            go_forward(27.5, 0,.2, false);
            turn_to_heading_pirouette(90, false);
            go_forward(11,90,.2, false);
            turn_to_heading_pirouette(120, false);
            go_forward(6.5,120, .2, false);
            sleep(300);
            lift_glyph("down");
            sleep(500);
            leftclamp.setPosition(GlobalVarriables.leftclampopen);
            rightclamp.setPosition(GlobalVarriables.rightclampopen);
            sleep(1000);
            go_forward(2,118,-.2, true);

        } else { // center
            go_forward(27.5, 0,.2, false);
            turn_to_heading_pirouette(124, false);
            go_forward(24,124,.2, false);
            sleep(300);
            lift_glyph("down");
            sleep(500);
            leftclamp.setPosition(GlobalVarriables.leftclampopen);
            rightclamp.setPosition(GlobalVarriables.rightclampopen);
            sleep(1000);
            go_forward(2,124,-.2, true);
        }

    }
}