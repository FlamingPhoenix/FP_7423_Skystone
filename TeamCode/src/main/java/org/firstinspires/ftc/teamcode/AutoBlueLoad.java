package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@Autonomous(name = "AutoBlueLoad", group = "none")
public class AutoBlueLoad extends AutoBase {

    @Override
    public void runOpMode() throws InterruptedException {

        float robotStartingAngle = initialize();

        waitForStart();
//        This code is assuming the camera is on the same side as the wheeled intake
//        Drive to left skystone

//        Pick up left skystone

//        Turn counter clockwise 90 degrees
        Turn(0.5F, 90, Direction.COUNTERCLOCKWISE, imu, this);
//        Drive forward x feet
        Drive(0.5F, 65, Direction.FORWARD);
        double driveDistance = DriveUntilDistance(3, robotStartingAngle, imu);
//        Drop left skystone

//        Drive back y feet
        Drive(0.5F, 89 + (float)driveDistance, Direction.BACKWARD);
//        Turn clockwise 90 degrees
        Turn(0.5F, 90, Direction.CLOCKWISE, imu, this);
//        Drive to right skystone

//        Pick up right skystone

//        Drive forward z feet

//        Drop right skystone

//        Drive backward x feet

//        Park under sky bridge

    }
}
