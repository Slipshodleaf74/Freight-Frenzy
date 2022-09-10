package org.firstinspires.ftc.teamcode.FirstTry;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name = "Lucas BOX Opmode", group = "TeleOp")
public class LucasCodeBOX extends LinearOpMode{

    //initialize opmode members
    LucasHardwareBOX robot = new LucasHardwareBOX();
    private ElapsedTime runtime = new ElapsedTime();

    // calculate these using measurement
    // encoder position variables
    private int sharedPos = 2900;
    private int midPos = 1950;
    private int highPos = 2200;
    private int capPos = 1700;
    private int vertPos = 1700;
    private int capGrabPos = 2525;

    private int extendMaxPos = -1400;
    private int extendCapPos = -450;
    private int extendMinPos = 0;
    private int extendFloorPos = -150;

    //initialize
    @Override
    public void runOpMode(){
        robot.init(hardwareMap);
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

        telemetry.addData("Mode", "calibrated!");
        telemetry.update();


        waitForStart();
        runtime.reset();

        robot.levelWrist();

        //variables for controlling
        double forwardleft, forwardright, strafeleft, straferight, armMove, extenderMove;

        //power variables
        double correction = 0; //dunno abt this one, maybe useful for autonomoosssss
        double armSpeed = 0.5;
        double armEncoderSpeed = 0.5;
        double carouselSpeed = 0.5;
        double grabOn = 0.09;
        double grabOff = 0.24;
        double extenderSpeed = 1;
        double intakeSpeed = -.8;

        double sens = 0.15;

        int previousArmPos = 0;

        boolean autoArm = false;
        boolean autoWrist = true;
        boolean autoExtend = false;
        boolean floored = false;

        boolean previousB1 = false;
        boolean previousA1 = false;

        boolean previousDpadDown2 = false;
        boolean previousLeftBumper2 = false;
        boolean previousRightBumper2 = false;
        boolean previousLeftStickButton2 =  false;
        boolean previousRightStickButton2 = false;

        int targetLevel = 0;

        //control loop
        while(opModeIsActive()){

            /*CONTROLS
            gamepad 1:
            Dpad - nudge wheels
            Left stick - left drive
            Right stick - right drive
            Bumpers - spin carousel
            a - pivot left from back
            b - pivot right from back
            y - cap pos auto move arm

            gamepad 2:
            Dpad up - reset encoder
            Dpad down - toggle to auto wrist movement
            Dpad horizontal - move wrist
            Left stick - extend arm
            Right stick - move arm
            Face buttons - auto move arm
                y - vert
                x - shared
                b - high pos
                a - home
            Left bumper - take in
            right bumper - take out
            right stick - toggle floor extension
             */

            //input gamepad joystick variables for control
            forwardleft = Range.clip(gamepad1.left_stick_y,-1,1);
            strafeleft = -1*Range.clip(gamepad1.left_stick_x,-1,1);
            forwardright = Range.clip(gamepad1.right_stick_y,-1,1);
            straferight = -1*Range.clip(gamepad1.right_stick_x, -1, 1);

            armMove = Range.clip(gamepad2.right_stick_y,-1,1);
            extenderMove = Range.clip(gamepad2.left_stick_y, -1, 1);


            //driving
            {
                //move wheels
                if (Math.abs(forwardleft) > sens || Math.abs(forwardright) > sens || Math.abs(strafeleft) > sens || Math.abs(straferight) > sens) {
                    robot.setDriveSpeeds(forwardleft, forwardright, strafeleft, straferight, correction);
                } else {
                    robot.stop();
                }

                //nudge wheels
                if (gamepad1.dpad_up) {
                    robot.setDriveSpeeds(-.3, -.3, 0, 0, correction);
                }
                if (gamepad1.dpad_down) {
                    robot.setDriveSpeeds(.3, .3, 0, 0, correction);
                }
                if (gamepad1.dpad_left) {
                    robot.setDriveSpeeds(0, 0, .3, .3, correction);
                }
                if (gamepad1.dpad_right) {
                    robot.setDriveSpeeds(0, 0, -.3, -.3, correction);
                }

                //pivot
                if(gamepad1.b && !previousB1){
                    //rotate right
                    robot.rotate(30, 0.5, this);

                    //wait for rotation, press other button to cancel
                    while(opModeIsActive() && Math.abs(robot.getAngle()) < 30){
                        if(gamepad1.a){
                            previousA1 = true;
                            break;
                        }
                    }

                    robot.backLeft.setPower(-robot.backLeft.getPower());
                    robot.backRight.setPower(-robot.backLeft.getPower());

                    sleep(50);

                    robot.stop();

                    // wait for rotation to stop.
                    //instance.sleep(1000);
                }
                previousB1 = gamepad1.b;

                if(gamepad1.a && !previousA1){
                    //rotate left
                    robot.rotate(-30, 0.5, this);

                    //wait for rotation, press other button to cancel
                    while(opModeIsActive() && Math.abs(robot.getAngle()) < 30){
                        if(gamepad1.b){
                            previousB1 = true;
                            break;
                        }
                    }

                    robot.backLeft.setPower(-robot.backLeft.getPower());
                    robot.backRight.setPower(-robot.backLeft.getPower());

                    sleep(50);

                    robot.stop();

                    // wait for rotation to stop.
                    //instance.sleep(1000);
                }
                previousA1 = gamepad1.a;
            }

            //manual arm control
            {
                //move arm
                if (Math.abs(armMove) > sens) {
                    robot.moveArm(-armMove * armSpeed);
                    autoArm = false;
                } else if (!autoArm) {
                    robot.moveArm(0);
                    //experimental funct
                    //robot.maintainArm();
                }

                //extend arm
                if (Math.abs(extenderMove) > sens) {
                    floored = false;
                    autoExtend = false;
                    robot.moveExtender(extenderMove);
                } else if (!autoExtend) {
                    robot.moveExtender(0);
                }
            }

            //automatic arm control
            {
                //reset encoder of arm and extender
                if (gamepad2.dpad_up) {
                    robot.arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    robot.arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

                    robot.extender.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    robot.extender.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                }

                //set arm target level and set mode to move
                if (gamepad2.y) {
                    //vert
                    targetLevel = vertPos;
                    autoArm = true;
                }
                if (gamepad2.x) {
                    //shared
                    targetLevel = sharedPos;
                    autoArm = true;
                }
                if (gamepad2.b) {
                    //high pos
                    targetLevel = highPos;
                    autoArm = true;
                }
                if (gamepad2.a) {
                    //home
                    targetLevel = 0;
                    autoArm = true;
                }

                //move arm if automatically moving, maintenance in here
                if (autoArm) {
                    robot.encoderMoveArm(targetLevel, armEncoderSpeed);

                    if(!robot.arm.isBusy()){
                        autoArm = false;
                        robot.arm.setPower(0);
                    }
                }

                //move arm to cap level automatically
                if (gamepad1.y) {
                    //set it to automatically move
                    autoArm = true;
                    autoWrist = false;
                    robot.saveWrist();

                    //set arm to move to cap pos
                    robot.encoderMoveArm(capPos, armEncoderSpeed);
                    while (robot.arm.isBusy() && opModeIsActive()) {
                        //pass
                    }

                    robot.encoderExtend(extendCapPos,extenderSpeed,true);

                    robot.levelWrist();
                }

                //extend arm to floor for intake "floored"
                if(gamepad2.right_stick_button && !previousRightStickButton2){
                    autoExtend = true;

                    if(floored){
                        floored = false;

                        robot.encoderExtend(0,extenderSpeed,false);
                    }
                    else{
                        floored = true;

                        robot.encoderExtend(extendFloorPos,extenderSpeed,false);
                    }
                }
                previousRightStickButton2 = gamepad2.right_stick_button;

                //stop arm from moving once it has reached its position
                if(autoExtend){
                    if(!robot.extender.isBusy()){
                        robot.extender.setPower(0);
                    }
                }
            }

            //misc functions, intake, carousel, wrist
            {
                //control intake
                if (gamepad2.left_bumper){
                    robot.intake(true,true);
                }
                else if (gamepad2.right_bumper){
                    robot.intake(true,false);
                }
                else{
                    robot.intake(false,false);
                }

                //Spin carousel
                if (gamepad1.right_bumper) {
                    robot.spin(carouselSpeed, true);
                } else if (gamepad1.left_bumper) {
                    robot.spin(carouselSpeed, false);
                } else {
                    robot.spin(0, true);
                }

                if(gamepad2.dpad_down){
                    robot.intakeWrist();
                    autoWrist = false;
                }
                else if(previousDpadDown2){
                    autoWrist = true;
                    //sleep to avoid spamming each loop
                }
                previousDpadDown2 = gamepad2.dpad_down;

                //wrist movement, now incorporated into the auto arm function
                if(autoWrist){
                    robot.levelWrist();
                }

                if(gamepad2.dpad_left){
                    autoWrist = false;
                    robot.moveWrist(false);
                }
                if(gamepad2.dpad_right){
                    autoWrist = false;
                    robot.moveWrist(true);
                }

                //telemetry
                telemetry.addData("Arm pos", robot.arm.getCurrentPosition());
                telemetry.addData("Ext pos", robot.extender.getCurrentPosition());
                telemetry.addData("Wrist Pos", robot.wrist.getPosition());
                telemetry.addData("Left trigger 1", Float.toString(gamepad1.left_trigger));
                telemetry.addData("Right trigger 1", Float.toString(gamepad1.right_trigger));
                telemetry.addData("Left trigger 2", Float.toString(gamepad2.left_trigger));
                telemetry.addData("Right trigger 2", Float.toString(gamepad2.right_trigger));
                telemetry.update();

                //previous buttons

            }

            //prev buttons
        }
    }
}
