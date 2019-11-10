package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "AutoRedLoad", group = "none")
public class AutoRedLoad extends AutoBase {
    @Override
    public void runOpMode() throws InterruptedException {

        initialize();

        waitForStart();
        Strafe(0.2F, 5, Direction.RIGHT);
        this.sleep(1000);
        StrafeToImage(0.1F, imageNavigation.stoneTarget, this, 12, 11, primaryAngle);
        this.sleep(1000);
        Strafe(0.1F, 3, Direction.LEFT);
        this.sleep(1000);
        Drive(0.1F, 1, Direction.FORWARD);
        this.sleep(1000);
        Turn(0.5F, 90, Direction.CLOCKWISE, imu, this);
        intakeMotorLeft.setPower(1);
        intakeMotorRight.setPower(1);
        this.sleep(1000);
        Drive(0.3F, 10, Direction.FORWARD);
        intakeMotorLeft.setPower(0);
        intakeMotorRight.setPower(0);
        this.sleep(1000);
        Drive(0.2F, 13, Direction.BACKWARD);
        this.sleep(1000);
        Turn(0.5F, 90, Direction.CLOCKWISE, imu, this);
        this.sleep(1000);
        Drive(0.5F, 60, Direction.FORWARD);
        intakeMotorLeft.setPower(-1);
        intakeMotorRight.setPower(-1);
        Drive(0.2F, 10, Direction.BACKWARD);
        intakeMotorLeft.setPower(0);
        intakeMotorRight.setPower(0);
    }
}
