package org.firstinspires.ftc.teamcode;

        import android.util.Log;

        import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
        import com.qualcomm.robotcore.hardware.ColorSensor;
        import com.qualcomm.robotcore.hardware.DcMotor;

        import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
        import org.firstinspires.ftc.teamcode.Library.ImageNavigation;
        import org.firstinspires.ftc.teamcode.Library.MyBoschIMU;

@Autonomous(name = "AutoBlueLoad", group = "none")
public class AutoBlueLoad extends AutoBase {

    @Override
    public void runOpMode() throws InterruptedException {

        initialize();
        waitForStart();

        Strafe(0.4F, 6F, Direction.RIGHT);

        this.sleep(600);

        resetAllEncoders();

        if (!StrafeToImage(0.4F, imageNavigation.stoneTarget, this, 8, 8, primaryAngle)) {
            Drive(0.2F, 8F, Direction.BACKWARD);

            resetAllEncoders();
            this.sleep(600);
            if (!StrafeToImage(0.4F, imageNavigation.stoneTarget, this, 8, 8, primaryAngle)) {
                Drive(0.2F, 6F, Direction.FORWARD);
                StrafeUntilDistance(0.3F, Direction.RIGHT, 5, primaryAngle, imu);

                int i = 0;

                while (!detectSkystoneByColor() && i < 2) {
                    Drive(0.2f, 7, Direction.BACKWARD);
                    i++;
                    telemetry.update();
                    this.sleep(350);
                }
                skystonePosition = i + 1;
                telemetry.update();
            }
            else {
                double distance = getLateralMovement();
                if (distance < -4) {
                    skystonePosition = 3;
                } else {
                    skystonePosition = 2;
                }
            }
        } else {
            double distance = getLateralMovement();
            if (distance < -4) {
                skystonePosition = 2;
            } else {
                skystonePosition = 1;
            }
        }

        Log.i("[phoenix]", String.format("skystonePosition = %d", skystonePosition));

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

        float distanceZ = 7;

        if(lastKnownPosition.translation != null)
            distanceZ = (distanceZ + (Math.abs(lastKnownPosition.translation.get(2)) / 25.4f - 3f)) * (float)Math.sqrt(2);
        Drive(0.2F, distanceZ, Direction.FORWARD);

        this.sleep(500);
        intakeMotorLeft.setPower(0);
        intakeMotorRight.setPower(0);

        Drive(0.8F, distanceZ, Direction.BACKWARD);

        imu.resetAndStart(Direction.COUNTERCLOCKWISE);

        float angleToBuild = primaryAngle - imu.getAngularOrientation().firstAngle;

        Log.i("[phoenix]", String.format("primary = %d; angleToBuild = %f10.2", primaryAngle, angleToBuild ));
        telemetry.addData("cac result", String.format("primary = %d; angleToBuild = %f10.2", primaryAngle, angleToBuild ));
        telemetry.update();

        Turn(0.2F, (int)angleToBuild, Direction.COUNTERCLOCKWISE, imu, this);

        this.sleep(100);

        //driving towards build zone
        Log.i("[phoenix:skystonePos]", String.format("skystonePosition=%d", skystonePosition));
        sleep(10000);
        double distanceToBuildZone = 50 + 8 * (skystonePosition - 1);
        Drive(0.5F, (float)distanceToBuildZone, Direction.FORWARD);

        intakeMotorLeft.setPower(-1);
        intakeMotorRight.setPower(-1);
        this.sleep(300);

        intakeMotorLeft.setPower(0);
        intakeMotorRight.setPower(0);

        Turn(0.4f, 179, Direction.CLOCKWISE, imu, this);
        Drive(0.5F, 50F, Direction.FORWARD);

        OpenGLMatrix coordinates = imageNavigation.getRobotLocation();

    }
}
