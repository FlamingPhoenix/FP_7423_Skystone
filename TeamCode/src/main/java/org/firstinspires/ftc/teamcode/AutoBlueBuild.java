package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@Autonomous(name = "AutoBlueBuild", group = "none")
public class AutoBlueBuild extends AutoBase {

    @Override
    public void runOpMode() throws InterruptedException {

        initialize();
        waitForStart();

        DriveUntilDistance(0.2F, Direction.BACKWARD, 4, primaryAngle, imu);
        puller.setPosition(1);
        this.sleep(1000);
        Drive(0.7F, 24, Direction.FORWARD);

        puller.setPosition(0);
        this.sleep(1000);

        Strafe(0.3F, 12, Direction.LEFT);
        Drive(0.3F, 24, Direction.BACKWARD);
        Strafe(0.3F, 40, Direction.RIGHT);
    }
}
