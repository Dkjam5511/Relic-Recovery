package org.firstinspires.ftc.teamcode.Tests;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Autonomous.Mecanum_Nav_Routines;
import org.firstinspires.ftc.teamcode.GlobalVarriables;

/**
 * Created by Drew on 2/18/2018.
 */
@Autonomous (name = "Triple Glyph Test", group = "Tests")
public class Triple_Glyph_Test extends Mecanum_Nav_Routines{

    @Override
    public void runOpMode() throws InterruptedException {
        MNav_Init();
        runtime.reset();
        go_forward(6, 0, -.5,false);
        turn_to_heading(180);
        rightclamp.setPosition(GlobalVarriables.rightclampopenpos);
        leftclamp.setPosition(GlobalVarriables.leftclampopenpos);
        setwheelintake(true, true, false);
        go_forward(9.5,180,1,false);
        rightclamp.setPosition(GlobalVarriables.rightclampclosedpos);
        leftclamp.setPosition(GlobalVarriables.leftclampclosedpos);
        sleep(100);
        go_forward(10, 180,-1, false);
        rightclamp.setPosition(GlobalVarriables.rightclampopenpos);
        leftclamp.setPosition(GlobalVarriables.leftclampopenpos);
        go_forward(11,180,.16, false);
        lift_glyph("up", 8, true);
        setwheelintake(false, true, true);
        go_forward(6, 180, -1, true);
        turn_to_heading(0);
        go_sideways(null, 90, 0, .7, .5, 0);
        go_forward(8,0,.35,false);
        leftclamp.setPosition(GlobalVarriables.leftclampinitpos);
        rightclamp.setPosition(GlobalVarriables.rightclampinitpos);
        sleep(300);
        go_forward(4, 0, -.2, true);
        lift_glyph("down", 0, false);
    }
}
