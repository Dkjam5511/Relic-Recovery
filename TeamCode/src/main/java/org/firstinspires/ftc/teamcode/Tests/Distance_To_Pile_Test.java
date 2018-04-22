package org.firstinspires.ftc.teamcode.Tests;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Autonomous.Mecanum_Nav_Routines;

@Autonomous(name = "Distance To Pile Test", group = "Tests")
public class Distance_To_Pile_Test extends Mecanum_Nav_Routines {

    double distancetopile;

    @Override
    public void runOpMode() throws InterruptedException {
        MNav_Init();
        runtime.reset();
        distancetopile = go_forward(15,0,.3,false);
        telemetry.addData("Distance To Pile: ", distancetopile);
        telemetry.update();
        sleep(5000);
    }
}
