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
        lift_glyph("up");
        vuforiareading = vuforia_scan();
        jewelknock("red");

        if (vuforiareading == "RIGHT"){
            go_forward(23, 0,.7);
            turn_to_heading(123);
            go_forward(26,123,.7);
            leftgrabber.setPosition(GlobalVarriables.leftgrabberopen);
            rightgrabber.setPosition(GlobalVarriables.rightgrabberopen);
            sleep(2000);

        } else if (vuforiareading == "LEFT"){
            go_forward(27.5, 0,.7);
            turn_to_heading(90);
            go_forward(13.5,90,.7);
            turn_to_heading(120);
            go_forward(10,120, .7);
            leftgrabber.setPosition(GlobalVarriables.leftgrabberopen);
            rightgrabber.setPosition(GlobalVarriables.rightgrabberopen);
            sleep(2000);
            lift_glyph("down");
            go_forward(1,120,-.7);
        } else {
            go_forward(27, 0,.7);
            turn_to_heading(116);
            go_forward(26,116,.7);
            leftgrabber.setPosition(GlobalVarriables.leftgrabberopen);
            rightgrabber.setPosition(GlobalVarriables.rightgrabberopen);
            sleep(2000);
            lift_glyph("down");
            go_forward(2,116,-.7);
        }

    }
}