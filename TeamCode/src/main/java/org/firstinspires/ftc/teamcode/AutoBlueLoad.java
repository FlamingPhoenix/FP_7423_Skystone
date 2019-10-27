package org.firstinspires.ftc.teamcode;

import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;

@Autonomous(name = "AutoBlueLoad", group = "none")
public class AutoBlueLoad extends AutoBase {

    @Override
    public void runOpMode() throws InterruptedException {

        initialize();

        waitForStart();

        Strafe(0.3f, 36, Direction.RIGHT);

        while(opModeIsActive())
        {
            OpenGLMatrix p = imageNavigation.getRobotLocation();

            if (p != null)
                Log.i("[phoenix]",  String.format("translateX = %10.2f, translateY = %10.2f, translateZ = %10.2f", p.getTranslation().get(0)/25.4, p.getTranslation().get(1)/25.4, p.getTranslation().get(2)/25.4));

        }



        //DriveToMatrix(-25F * 25.4F, 24F * 25.4F);
    }
}
