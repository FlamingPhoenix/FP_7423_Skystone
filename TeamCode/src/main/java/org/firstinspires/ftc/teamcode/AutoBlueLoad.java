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
        Turn(0.5F, 90, Direction.COUNTERCLOCKWISE, imu, this);
        Drive(0.5F, 60, Direction.FORWARD);
        intakeMotorLeft.setPower(-1);
        intakeMotorRight.setPower(-1);

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
