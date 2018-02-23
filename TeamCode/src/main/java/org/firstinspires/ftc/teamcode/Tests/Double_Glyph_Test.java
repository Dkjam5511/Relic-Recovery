package org.firstinspires.ftc.teamcode.Tests;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Autonomous.Mecanum_Nav_Routines;
import org.firstinspires.ftc.teamcode.GlobalVarriables;

/**
 * Created by Drew on 2/18/2018.
 */
@Autonomous (name = "Double Glyph Test", group = "Tests")
public class Double_Glyph_Test extends Mecanum_Nav_Routines{

    @Override
    public void runOpMode() throws InterruptedException {
        MNav_Init();
        runtime.reset();
        rightclamp.setPosition(GlobalVarriables.rightclampopenpos);
        leftclamp.setPosition(GlobalVarriables.leftclampopenpos);
        turn_to_heading(180);
        setwheelintake(false, true);
        go_forward(22,180,.2,false);
        turn_to_heading(195);
        setwheelintake(true, true);
        turn_to_heading(180);
        go_forward(8,180,.15, false);
        lift_glyph("up", 3, true);
        setwheelintake(false, false);
        go_forward(16, 180, -.5, true);
        turn_to_heading(0);
        lift_glyph("up", 12, true);
        go_forward(16,0,.3,false);
        leftclamp.setPosition(GlobalVarriables.leftclampopenpos);
        rightclamp.setPosition(GlobalVarriables.rightclampopenpos);
        setwheelintake(false, true);
        sleep(400);
        go_forward(3, 0, -.2, true);
        lift_glyph("down", 0, false);

    }
}
