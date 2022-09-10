package org.firstinspires.ftc.teamcode.FirstTry;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

@com.qualcomm.robotcore.eventloop.opmode.Autonomous(name = "measuring code")
public class Measurement extends LinearOpMode{
    AutoHardwareBOXNOWRIST robot = new AutoHardwareBOXNOWRIST();

    @Override
    public void runOpMode() throws InterruptedException {
        robot.init(hardwareMap);

        //undo init() setting all motors to brake
        robot.arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        robot.extender.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        robot.frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        robot.frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        robot.backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        robot.backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();
        while(opModeIsActive()){
            telemetry.addData("FR wheel position:", Integer.toString(robot.frontRight.getCurrentPosition()));
            telemetry.addData("FL wheel position:", Integer.toString(robot.frontLeft.getCurrentPosition()));
            telemetry.addData("BR wheel position:", Integer.toString(robot.backRight.getCurrentPosition()));
            telemetry.addData("BL wheel position:", Integer.toString(robot.backLeft.getCurrentPosition()));
            telemetry.addData("arm position:", Integer.toString(robot.arm.getCurrentPosition()));
            telemetry.addData("extender position", Integer.toString(robot.extender.getCurrentPosition()));
            //telemetry.addData("servo 1 position", Double.toString(robot.grab1.getPosition()));
            //telemetry.addData("Wrist position", Double.toString(robot.wrist.getPosition()));
            telemetry.addData("angle:", Double.toString(robot.getAngle()));
            //telemetry.addData("flipper", Double.toString(robot.flipper.getPosition()));
            telemetry.update();
        }
    }
}
