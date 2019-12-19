package org.firstinspires.ftc.teamcode;

        import android.util.Log;

        import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
        import com.qualcomm.robotcore.hardware.ColorSensor;

        import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
        import org.firstinspires.ftc.teamcode.Library.MyBoschIMU;

@Autonomous(name = "AutoBlueLoad", group = "none")
public class AutoBlueLoad extends AutoBase {

    @Override
    public void runOpMode() throws InterruptedException {

        initialize();
        waitForStart();

        Strafe(0.3F, 9F, Direction.RIGHT);

        this.sleep(350);
        if (!StrafeToImage(0.3F, imageNavigation.stoneTarget, this, 8, 7, primaryAngle)) {
            Drive(0.2F, 6F, Direction.BACKWARD);

            this.sleep(350);
            if (!StrafeToImage(0.3F, imageNavigation.stoneTarget, this, 8, 7, primaryAngle)) {
                Drive(0.2F, 6F, Direction.FORWARD);
                StrafeUntilDistance(0.45F, Direction.RIGHT, 10, primaryAngle, imu);

                int i = 0;

                while (!detectSkystoneByColor() && i < 2) {
                    Drive(0.2f, 7, Direction.BACKWARD);
                    i++;
                    telemetry.update();
                    this.sleep(350);
                }
                telemetry.update();
            }
        }
        float distanceX = 6;
//        distanceX = distanceX - getSkystoneXPosition(imageNavigation.stoneTarget);
        if(lastKnownPosition.translation != null)
            distanceX = distanceX - lastKnownPosition.translation.get(1) / 25.4f;
        telemetry.addData("DistanceX: ", "%f", distanceX);
        telemetry.update();

        Drive(0.2F, distanceX, Direction.BACKWARD);
        this.sleep(100);
        Turn(0.2F, 45, Direction.CLOCKWISE, imu, this);
        this.sleep(100);
        intakeMotorLeft.setPower(1);
        intakeMotorRight.setPower(1);

        float distanceZ = 6;

        if(lastKnownPosition.translation != null)
            distanceZ = (distanceZ + (Math.abs(lastKnownPosition.translation.get(2)) / 25.4f - 3f)) * (float)Math.sqrt(2);
        Drive(0.2F, distanceZ, Direction.FORWARD);

        this.sleep(500);
        intakeMotorLeft.setPower(0);
        intakeMotorRight.setPower(0);

        Drive(0.2F, distanceZ, Direction.BACKWARD);

        float angleToBuild = primaryAngle - imu.getAngularOrientation().firstAngle;
        Turn(0.2F, (int)angleToBuild, Direction.COUNTERCLOCKWISE, imu, this);

        this.sleep(100);

        Strafe(0.3F, 9F, Direction.LEFT);

        this.sleep(100);

        Drive(0.5F, 50F, Direction.FORWARD);

        intakeMotorLeft.setPower(-1);
        intakeMotorRight.setPower(-1);
        this.sleep(300);

        intakeMotorLeft.setPower(0);
        intakeMotorRight.setPower(0);

        Drive(0.3F, 10, Direction.BACKWARD);
    }
}
