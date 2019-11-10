package org.firstinspires.ftc.teamcode;

import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.teamcode.Library.MyBoschIMU;

@Autonomous(name = "AutoBlueLoad", group = "none")
public class AutoBlueLoad extends AutoBase {

    @Override
    public void runOpMode() throws InterruptedException {

        initialize();

        waitForStart();

        while (true) {
            float location = imageNavigation.getSkystoneLocationByTf();
            telemetry.addData("Skystone Location:", "%f", location);
            telemetry.update();
            this.sleep(1000);
        }

//        if (!StrafeToImage(0.1F, imageNavigation.stoneTarget, this, 8, 11, primaryAngle))
//            StrafeUntilDistance(5, primaryAngle, imu);


//        this.sleep(1000);
//        Strafe(0.1F, 3, Direction.LEFT);
//        this.sleep(1000);
//        Drive(0.1F, 1, Direction.FORWARD);
//        this.sleep(1000);
//        Turn(0.5F, 90, Direction.CLOCKWISE, imu, this);
//        intakeMotorLeft.setPower(1);
//        intakeMotorRight.setPower(1);
//        this.sleep(1000);
//        Drive(0.3F, 10, Direction.FORWARD);
//        intakeMotorLeft.setPower(0);
//        intakeMotorRight.setPower(0);
//        this.sleep(1000);
//        Drive(0.2F, 13, Direction.BACKWARD);
//        this.sleep(1000);
//        Turn(0.5F, 90, Direction.COUNTERCLOCKWISE, imu, this);
//        this.sleep(1000);
//        Drive(0.5F, 60, Direction.FORWARD);
//        intakeMotorLeft.setPower(-1);
//        intakeMotorRight.setPower(-1);
//        Drive(0.2F, 10, Direction.BACKWARD);
//        intakeMotorLeft.setPower(0);
//        intakeMotorRight.setPower(0);




      /*ACTUAL CODE*/

//        while(opModeIsActive())
//        {
//            OpenGLMatrix robotLocation = imageNavigation.getRobotLocation();
//            Log.i("[phoenix]",  String.format("translateX = %10.2f, translateY = %10.2f, translateZ = %10.2f", robotLocation.getTranslation().get(0)/25.4, robotLocation.getTranslation().get(1)/25.4, robotLocation.getTranslation().get(2)/25.4));
//        }


//        Strafe(0.3F, 6, Direction.RIGHT);
//
//
//        OpenGLMatrix p = imageNavigation.getRobotLocation();
//        double skystoneX = p.getTranslation().get(0)/25.4;
//        Skystone skystonePos;
//
//        if(skystoneX > -4) {
//            skystonePos = Skystone.LEFT;
//        }
//
//        else if (skystoneX < -4) {
//            skystonePos = Skystone.CENTER;
//        }
//
//        else {
//            skystonePos = Skystone.RIGHT;
//        }
//
//        Turn(0.5f, 90, Direction.CLOCKWISE, imu, this);
//        if(skystonePos == Skystone.LEFT) {
//            DriveToCoordinate(-28F * 25.4F, 24F * 25.4F);
//
//        }
//
//        else if(skystonePos == Skystone.CENTER) {
//            DriveToCoordinate(-36F * 25.4F, 24F * 25.4F);
//
//        }
//
//        else {
//            DriveToCoordinate(-44F * 25.4F, 24F * 25.4F);
//        }
    }
}
