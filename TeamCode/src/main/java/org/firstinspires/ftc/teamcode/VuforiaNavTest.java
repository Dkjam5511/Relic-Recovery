package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

/**
 * Created by Drew on 10/2/2017.
 */
@Autonomous (name = "VuforiaNavTest", group = "Test")
public class VuforiaNavTest extends Navigation_Routines  {
    @Override
    public void runOpMode() throws InterruptedException {
            waitForStart();
            vuforia_scan();
        while(opModeIsActive()) {
            telemetry.addLine("Picture Reading Returned");
            telemetry.addData("Picture Reading:", picturereading);
            telemetry.update();
        }
    }
}
