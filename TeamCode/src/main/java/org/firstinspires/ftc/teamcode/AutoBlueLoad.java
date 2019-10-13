package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@Autonomous(name = "AutoBlueLoad", group = "none")
public class AutoBlueLoad extends AutoBase {

    @Override
    public void runOpMode() throws InterruptedException {

        initialize();

        waitForStart();

//        Scan first three blocks to find Skystones; Drive to left skystone
        StrafeToImage(0.5F, stoneTarget, this, 2);

//        Pick up left skystone

//        Turn counter clockwise 90 degrees

//        Strafe left

//        Drive forward x feet

//        Drop left skystone

//        Drive back y feet

//        Turn clockwise 90 degrees

//        Drive to right skystone

//        Pick up right skystone

//        Drive forward z feet

//        Drop right skystone

//        Drive backward x feet

//        Park under sky bridge

    }
}
