package org.firstinspires.ftc.teamcode.FirstTry;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

@com.qualcomm.robotcore.eventloop.opmode.Autonomous(name = "RC archive")
public class RCarchive extends LinearOpMode{
    AutoHardware robot = new AutoHardware();

    @Override
    public void runOpMode() throws InterruptedException {
        robot.init(hardwareMap);

        robot.grab(1);

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        telemetry.addData("Mode", "calibrating...");
        telemetry.update();

        // make sure the imu gyro is calibrated before continuing.
        while (!isStopRequested() && !robot.imu.isGyroCalibrated())
        {
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
        robot.rotate(-125,.5,this);

        //move arm up to level
        robot.moveArm(3,.2,this, true);
        sleep(500);

        //move forward?
        //robot.driveAndStop(dontKnow, 1, this);

        //release
        robot.grab(0);
        sleep(1000);

        //move back
        //robot.driveAndStop(-dontKnow, 1, this);

        //return arm
        robot.moveArm(4,.2,this, true);

        //rotate to go to carousel
        robot.rotate(-10,.5,this);

        //drive to carousel
        robot.driveAndStop(35, .6,this, true);
        //slow down a lil
        robot.driveToPosition(4,.2,this, true);

        //spin carousel
        robot.spin(.5,false);
        sleep(5000);
        robot.spin(.0,true);

        //strafe towards park

        //rotate to park
        robot.rotate(35,.5,this);

        //strafe to park
        robot.strafeToPosition(-25,.4,this, true);

        //move back
        robot.driveAndStop(-5,.4,this, true);

        //return arm
        robot.moveArm(0,.2,this, true);

        //drive into park
        robot.driveAndStop(5,.4,this, true);

        //stop robot
        robot.stop();
    }
}
