package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Autonomous.Mecanum_Nav_Routines;
import org.firstinspires.ftc.teamcode.GlobalVarriables;

/**
 * Created by Drew on 2/1/2018.
 */
@Autonomous(name = "Bottom Blue Mecanum", group = "Autonomous")
public class Bottom_Blue_Mecanum extends Mecanum_Nav_Routines {

    String[] vuforiareading = new String[2];

    @Override
    public void runOpMode() throws InterruptedException {
        MNav_Init();
        runtime.reset();
        lift_glyph("up", 5.5, true);
        vuforiareading = vuforia_scan();
        jewelknockvuforia("blue", vuforiareading[1], false);

        go_forward(26, 0, .12, false);
        turn_to_heading(270);
        lift_glyph("up", 15, false);
        go_sideways(null, 270, 270, .27, 2, 0);
        go_forward(2,270,.2, false);
        wall_distance_align(16);
        sleep(500);
        go_sideways("blue", 90, 270, .27, 10, 16);

        if (vuforiareading[0] == "LEFT") {
            go_sideways(null, 270, 270, .27, 2, 0);
        } else if (vuforiareading[0] == "RIGHT") {
            go_sideways(null, 90, 270, .27, 2.2, 0);
        } else {
            go_sideways(null, 90, 270, .27, .2, 0);
        }
        lift_glyph("down", 3,false);
        go_forward(7, 270, .15, false);
        sleep(300);
        lift_glyph("down", 0, false);
        sleep(500);
        leftclamp.setPosition(GlobalVarriables.leftclampopenpos);
        rightclamp.setPosition(GlobalVarriables.rightclampopenpos);
        sleep(1000);
        go_forward(4, 270,-.2, true);

    }
}
