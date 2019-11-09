package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "AutoRedLoad", group = "none")
public class AutoRedLoad extends AutoBase {
    @Override
    public void runOpMode() throws InterruptedException {

        initialize();

        waitForStart();

        Strafe(0.2F, 2, Direction.RIGHT);
        this.sleep(2000);
        StrafeToImage(0.1F, imageNavigation.stoneTarget, this, 0, 11);
        Turn(0.5F, 90, Direction.CLOCKWISE, imu, this);
        intakeMotorLeft.setPower(1);
        intakeMotorRight.setPower(1);
        Drive(0.6F, 5, Direction.FORWARD);
        intakeMotorLeft.setPower(0);
        intakeMotorRight.setPower(0);
        Drive(0.2F, 8, Direction.BACKWARD);
        Turn(0.5F, 90, Direction.CLOCKWISE, imu, this);
        Drive(0.5F, 60, Direction.FORWARD);
        intakeMotorLeft.setPower(-1);
        intakeMotorRight.setPower(-1);
    }
}
