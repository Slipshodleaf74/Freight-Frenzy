package org.firstinspires.ftc.teamcode.FirstTry;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

@com.qualcomm.robotcore.eventloop.opmode.Autonomous(name = "BW archive")
public class BWarchive extends LinearOpMode{
    AutoHardware robot = new AutoHardware();

    @Override
    public void runOpMode() throws InterruptedException {
        robot.init(hardwareMap);

        //robot.grab();

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        telemetry.addData("Mode", "calibrating...");
        telemetry.update();

        // make sure the imu gyro is calibrated before continuing.
        while (!isStopRequested() && !robot.imu.isGyroCalibrated()) {
            sleep(50);
            idle();
        }

        robot.frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        telemetry.addData("Mode", "calibrated");
        telemetry.update();

        waitForStart();

        //movements for auto
        //move forward
        robot.driveAndStop(28, .6, this, true);

        sleep(500);

        //rotate
        robot.rotate(-125, .5, this);

        //move arm up to level
        robot.moveArm(3, .2, this, true);
        sleep(500);

        //move forward?
        //robot.driveAndStop(dontKnow, 1, this);

        //release
        //robot.release();
        sleep(1000);

        //move back
        //robot.driveAndStop(-dontKnow, 1, this);

        //hold arm above
        robot.moveArm(4,.2,this, true);

        sleep(1000);

        //move forward to avoid bar
        robot.driveAndStop(4,.6,this, true);

        sleep(500);

        //rotate to park
        robot.rotate(35, .5, this);


        sleep(500);

        robot.strafeToPosition(7,.4,this, true);

        sleep(500);

        //drive to park zone
        robot.driveAndStop(45, 1, this, true);

        //strafe to make way
        robot.strafeToPosition(-10, .4,this, true);

        robot.strafeToPosition(5,.4,this, true);

        //return arm
        robot.moveArm(0, .2, this, true);

        //stop robot
        robot.stop();
    }
}
