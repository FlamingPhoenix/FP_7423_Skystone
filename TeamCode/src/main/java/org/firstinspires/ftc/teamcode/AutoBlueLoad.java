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

//        while(opModeIsActive())
//        {
//            OpenGLMatrix robotLocation = imageNavigation.getRobotLocation();
//            Log.i("[phoenix]",  String.format("translateX = %10.2f, translateY = %10.2f, translateZ = %10.2f", robotLocation.getTranslation().get(0)/25.4, robotLocation.getTranslation().get(1)/25.4, robotLocation.getTranslation().get(2)/25.4));
//        }


        Strafe(0.3F, 6, Direction.RIGHT);


        OpenGLMatrix p = imageNavigation.getRobotLocation();
        double skystoneX = p.getTranslation().get(0)/25.4;
        Skystone skystonePos;

        if(skystoneX > -4) {
            skystonePos = Skystone.LEFT;
        }

        else if (skystoneX < -4) {
            skystonePos = Skystone.CENTER;
        }

        else {
            skystonePos = Skystone.RIGHT;
        }

        Turn(0.5f, 90, Direction.CLOCKWISE, imu, this);
        if(skystonePos == Skystone.LEFT) {
            DriveToMatrix(-28F * 25.4F, 24F * 25.4F);

        }

        else if(skystonePos == Skystone.CENTER) {
            DriveToMatrix(-36F * 25.4F, 24F * 25.4F);

        }

        else {
            DriveToMatrix(-44F * 25.4F, 24F * 25.4F);
        }
    }
}
