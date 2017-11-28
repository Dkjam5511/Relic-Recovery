package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Navigation_Routines;
import org.firstinspires.ftc.teamcode.GlobalVarriables;

/**
 * Created by Drew on 10/25/2017.
 */
@Autonomous (name = "Bottom_Red", group = "Autonomous")
public class Bottom_Red extends Navigation_Routines {

    String vuforiareading;

    @Override
    public void runOpMode() throws InterruptedException {
        NAV_init();
        runtime.reset();
        lift_glyph("up");
        vuforiareading = vuforia_scan();
        jewelknock("red");
        turn_to_heading_pirouette(35);
        turn_to_heading_pirouette(70);
        go_forward(2,70,.6);
        turn_to_heading_pirouette(90);

        if (vuforiareading == "LEFT"){
            go_forward(13.5, 90,.7);
            turn_to_heading(180);
           go_forward(4,180,.5);
            leftclamp.setPosition(GlobalVarriables.leftclampopen);
            rightclamp.setPosition(GlobalVarriables.rightclampopen);
            sleep(2000);
            lift_glyph("down");
            go_forward(1,180,-.7);
        } else if (vuforiareading == "RIGHT"){
            go_forward(1.5, 90,.7);
            turn_to_heading(180);
            go_forward(4,180,.7);
            leftclamp.setPosition(GlobalVarriables.leftclampopen);
            rightclamp.setPosition(GlobalVarriables.rightclampopen);
            sleep(2000);
            lift_glyph("down");
            go_forward(1,180,-.7);
        } else{
            go_forward(7,90,.7);
            turn_to_heading(180);
            go_forward(4,180,.7);
            leftclamp.setPosition(GlobalVarriables.leftclampopen);
            rightclamp.setPosition(GlobalVarriables.rightclampopen);
            sleep(2000);
            lift_glyph("down");
            sleep(200);
            go_forward(1,180,-.7);
        }
    }
}
