package org.firstinspires.ftc.teamcode.Tests;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Autonomous.Mecanum_Nav_Routines;

/**
 * Created by Drew on 1/10/2018.
 */
@Autonomous (name = "Wall_Distance_Align", group = "Tests")
public class Wall_Distance_Align extends Mecanum_Nav_Routines{

    @Override
    public void runOpMode() throws InterruptedException {
        MNav_Init();
        runtime.reset();
        lift_glyph("up", 15, false);
        wall_distance_align(16);
    }
}