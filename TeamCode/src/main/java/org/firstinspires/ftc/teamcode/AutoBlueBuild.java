package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@Autonomous(name = "AutoBlueBuild", group = "none")
public class AutoBlueBuild extends AutoBase {

    @Override
    public void runOpMode() throws InterruptedException {

        initialize();
        waitForStart();

        DriveUntilDistance(0.2F, Direction.BACKWARD, 6, primaryAngle, imu);
    }
}
