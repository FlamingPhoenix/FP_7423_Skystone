package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "AutoRedBuild", group = "none")
public class AutoRedBuild extends AutoBase {

    @Override
    public void runOpMode() throws InterruptedException {

        initialize();
        waitForStart();

        Drive(0.2F, 5, Direction.FORWARD);
        Strafe(0.45F, 3, Direction.LEFT);

    }
}
