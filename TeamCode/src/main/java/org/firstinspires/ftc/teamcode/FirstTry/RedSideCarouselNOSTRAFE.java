package org.firstinspires.ftc.teamcode.FirstTry;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

@com.qualcomm.robotcore.eventloop.opmode.Autonomous(name = "Red Side Carousel NOSTRAFE")
public class RedSideCarouselNOSTRAFE extends LinearOpMode{
    AutoHardwareBOXNOWRIST robot = new AutoHardwareBOXNOWRIST();

    //auto arm and extender position variables
    int highArm = 1800;
    int midArm = 2000;
    int lowArm = 2250;

    int highExt = -800;
    int midExt = -600;
    int lowExt = -500;

    //power variables
    double driveSpeed = 0.6;
    double strafeSpeed = 0.5;
    double rotateSpeed = 0.5;
    double armSpeed = 0.4;
    double carouselSpeed = 0.5;
    double extendSpeed = 0.8;

    double grabOff = 0.24;
    double grabOn = 0.15;

    //target level variable
    int level = 0;
    int extLevel = 0;

    //variables for camera operation
    //row, column (y, x)
    int[] subMatZero = {110,10};
    int[] subMatOne = {110,240};
    int[] subMatTwo = {110,470};

    int subMatWidth = 30;
    int subMatHeight = 40;

    static final int STREAM_WIDTH = 640; // modify for your camera
    static final int STREAM_HEIGHT = 360; // modify for your camera
    OpenCvWebcam webcam;
    FreightFrenzyPipeline pipeline;

    int position = 0;
    //end of variables for camera

    public void telemetry(){
        telemetry.addData("wheel position:", Integer.toString(robot.frontRight.getCurrentPosition()));
        telemetry.addData("arm position:", Integer.toString(robot.arm.getCurrentPosition()));
        telemetry.addData("extender position", Integer.toString(robot.extender.getCurrentPosition()));
        telemetry.addData("angle:", Double.toString(robot.getAngle()));
        telemetry.update();
    }

    @Override
    public void runOpMode() throws InterruptedException {
        robot.init(hardwareMap);

        //initialize camera
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        WebcamName webcamName = null;
        webcamName = hardwareMap.get(WebcamName.class, "webcam"); // put your camera's name here
        webcam = OpenCvCameraFactory.getInstance().createWebcam(webcamName, cameraMonitorViewId);
        pipeline = new FreightFrenzyPipeline(subMatWidth,subMatHeight,subMatZero,subMatOne,subMatTwo);
        webcam.setPipeline(pipeline);
        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                webcam.startStreaming(STREAM_WIDTH, STREAM_HEIGHT, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {
                telemetry.addData("Camera Failed","");
                telemetry.update();
            }
        }); //done initializing camera

        //telemetry communication
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

        telemetry.addData("Mode", "calibrated");
        telemetry.update();

        while (!isStarted()){
            position = pipeline.position;
            telemetry.addData("Position", position);
            telemetry.update();
        }

        //no more need for camera beyond this point
        webcam.stopStreaming();

        //redundant
        //waitForStart();
        //calculate target based on position of marker

        if(position == 0){
            //left position, lowest level
            level = lowArm;
            extLevel = lowExt;
        }
        else if(position == 1){
            //middle position, middle level
            level = midArm;
            extLevel = midExt;
        }
        else if (position == 2){
            //right position, highest level
            level = highArm;
            extLevel = highExt;
        }

        //movements for auto
        //drive forward
        robot.driveAndStop(20,driveSpeed,this, true);

        sleep(500);

        //rotate and move arm at the same time
        robot.moveArm(level,armSpeed,this,false);

        robot.rotate(-135,rotateSpeed,this);

        while(robot.arm.isBusy() && opModeIsActive()){
            //wait
        }

        //brake robot arm
        robot.maintainArm();

        sleep(500);

        //extend arm
        robot.moveExtender(extLevel,extendSpeed);

        robot.maintainArm();

        //release
        robot.intake(true,false);
        sleep(500);
        robot.intake(false,false);

        //retract arm, extender, and reset grabber
        robot.moveArm(0, armSpeed, this, false);

        robot.moveExtender(0, extendSpeed);

        sleep(200);

        //rotate to align with wall (turn -90 overall)
        robot.rotate(45,rotateSpeed,this);

        sleep(500);

        //drive forward near carousel
        robot.driveAndStop(30,driveSpeed,this,true);

        robot.rotate(-45,rotateSpeed,this);

        while(robot.arm.isBusy() && opModeIsActive()){
            //wait for arm to finally finish
        }

        sleep(300);

        //drive into carousel
        robot.driveToPosition(15,driveSpeed/2,this, true);

        //spin carousel
        robot.spin(carouselSpeed,false);

        sleep(3000);

        robot.spin(0,false);

        //drive backwards
        robot.driveAndStop(-15,driveSpeed,this, true);

        //rotate to park
        robot.rotate(90,rotateSpeed,this);

        //drive into parking space
        robot.driveAndStop(20,driveSpeed,this,true);

        //rotate for better park
        robot.rotate(135,rotateSpeed,this);

        //drive back into wall
        robot.driveToPosition(-5,driveSpeed,this, true);

        //stop
        robot.stop();
    }
}
