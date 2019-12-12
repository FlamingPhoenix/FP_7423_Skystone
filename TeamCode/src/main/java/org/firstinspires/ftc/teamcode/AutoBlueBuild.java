package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@Autonomous(name = "AutoBlueBuild", group = "none")
public class AutoBlueBuild extends AutoBase {

    @Override
    public void runOpMode() throws InterruptedException {

        initialize();
        waitForStart();

        DriveUntilDistance(0.3f, Direction.BACKWARD, 6, primaryAngle, imu);

        pullerLeft.setPosition(1);
        pullerRight.setPosition(1);

        Drive(0.3f, 20, Direction.FORWARD);
    }
}
