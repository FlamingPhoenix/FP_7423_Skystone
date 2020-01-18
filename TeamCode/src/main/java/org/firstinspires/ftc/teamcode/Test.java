package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "Test", group = "none")
public class Test extends AutoBase {
    @Override
    public void runOpMode() throws InterruptedException {
        while(opModeIsActive()) {
            int red = colorSensor.red();
            int blue = colorSensor.blue();
            int green = colorSensor.green();

            telemetry.addData("Colors: ", String.format("red: %5d, green: #5d, blue: %5d", red, green, blue));

            telemetry.update();
        }

    }
}
