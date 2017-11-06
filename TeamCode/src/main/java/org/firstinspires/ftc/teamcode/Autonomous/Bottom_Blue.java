package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Navigation_Routines;
import org.firstinspires.ftc.teamcode.GlobalVarriables;

/**
 * Created by Drew on 10/25/2017.
 */
@Autonomous (name = "Bottom_Blue", group = "Autonomous")
public class Bottom_Blue extends Navigation_Routines {

    String vuforiareading;

    @Override
    public void runOpMode() throws InterruptedException {
        NAV_init();
        lift_glyph("up");
        vuforiareading = vuforia_scan();
        jewelknock("blue");
        turn_to_heading_pirouette(325);
        turn_to_heading_pirouette(290);
        go_forward(2,290,.6);
        turn_to_heading_pirouette(270);

        if (vuforiareading == "LEFT"){
            go_forward(13.5, 270,.7);
            turn_to_heading(180);
            go_forward(4,180,.5);
            leftgrabber.setPosition(GlobalVarriables.leftgrabberopen);
            rightgrabber.setPosition(GlobalVarriables.rightgrabberopen);
            sleep(2000);
            lift_glyph("down");
            go_forward(1,180,-.7);
        } else if (vuforiareading == "RIGHT"){
            go_forward(1.5, 270,.7);
            turn_to_heading(180);
            go_forward(4,180,.7);
            leftgrabber.setPosition(GlobalVarriables.leftgrabberopen);
            rightgrabber.setPosition(GlobalVarriables.rightgrabberopen);
            sleep(2000);
            lift_glyph("down");
            go_forward(1,180,-.7);
        } else{
            go_forward(6,270,.7);
            turn_to_heading(180);
            go_forward(4,180,.7);
            leftgrabber.setPosition(GlobalVarriables.leftgrabberopen);
            rightgrabber.setPosition(GlobalVarriables.rightgrabberopen);
            sleep(2000);
            lift_glyph("down");
            sleep(200);
            go_forward(1,180,-.7);
        }
    }
}
