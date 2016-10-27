package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.matrices.MatrixF;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;
import org.firstinspires.ftc.teamcode.robot.HardwareCodeBot;

import java.util.ArrayList;
import java.util.List;

/**
 * This file contains an example of an iterative (Non-Linear) "OpMode".
 * An OpMode is a 'program' that runs in either the autonomous or the teleop period of an FTC match.
 * The names of OpModes appear on the menu of the FTC Driver Station.
 * When an selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 * <p>
 * This particular OpMode just executes a basic Tank Drive Teleop for a PushBot
 * It includes all the skeletal structure that all iterative OpModes contain.
 * <p>
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@Autonomous(name = "MsToebes Autonomous: Iterative", group = "Autonomous")
// @Autonomous(...) is the other common choice
public class MsToebesAutonomous extends OpMode {
    public static float mmPerInch = 25.4f;
    public static float mmFTCFieldWidth  = (12*12 - 2) * mmPerInch;   // the FTC field is ~11'10" center-to-center of the glass panels
    public static float mmFTCTileWidth = 12 * 2 * mmPerInch;
    public static float mmPictureHieght =  8.5f * mmPerInch;
    public static float mmFloorClearance = 1.5f * mmPerInch;

    public static final String TAG = "Example";

    HardwareCodeBot robot = new HardwareCodeBot();
    int majorStep = 0;
    int minorStep = 0;
    int nTimes = 0;

    /**
     * {@link #vuforia} is the variable we will use to store our instance of the Vuforia
     * localization engine.
     */
    VuforiaLocalizer vuforia;
    List<VuforiaTrackable> allTrackables;
    OpenGLMatrix toolsLocationMatrix, gearsLocationMatrix,
                 wheelsLocationMatrix,legosLocationMatrix ;
    VuforiaTrackables ftcAssets;

    OpenGLMatrix lastLocation = null;

    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {

        robot.init(hardwareMap);

        /**
         * Start up Vuforia, telling it the id of the view that we wish to use as the parent for
         * the camera monitor feedback; if no camera monitor feedback is desired, use the parameterless
         * constructor instead. We also indicate which camera on the RC that we wish to use. For illustration
         * purposes here, we choose the back camera; for a competition robot, the front camera might
         * prove to be more convenient.
         *
         * Note that in addition to indicating which camera is in use, we also need to tell the system
         * the location of the phone on the robot; see phoneLocationOnRobot below.
         */

        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(com.qualcomm.ftcrobotcontroller.R.id.cameraMonitorViewId);
        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;
        this.vuforia = ClassFactory.createVuforiaLocalizer(parameters);

        /**
         * Load the data sets that for the trackable objects we wish to track. These particular data
         * sets are stored in the 'assets' part of our application (you'll see them in the Android
         * Studio 'Project' view over there on the left of the screen). You can make your own datasets
         * with the Vuforia Target Manager: https://developer.vuforia.com/target-manager.
         */
        ftcAssets = this.vuforia.loadTrackablesFromAsset("FTC_2016-17");
        VuforiaTrackable wheelsAsset = ftcAssets.get(0);
        wheelsAsset.setName("Wheels");
        VuforiaTrackable toolsAsset = ftcAssets.get(1);
        toolsAsset.setName("Tools");
        VuforiaTrackable legosAsset = ftcAssets.get(2);
        legosAsset.setName("Legos");
        VuforiaTrackable gearsAsset = ftcAssets.get(3);
        gearsAsset.setName("Gears");

        /** For convenience, gather together all the trackable objects in one easily-iterable collection */
        allTrackables = new ArrayList<VuforiaTrackable>();
        allTrackables.addAll(ftcAssets);

        // Location matrix for Tools RED Beacon
        OpenGLMatrix r = Orientation.getRotationMatrix(
                AxesReference.EXTRINSIC, /* using the world (field) coordinate system) */
                AxesOrder.XZX,
                AngleUnit.DEGREES, 90 /* 1st rotate CCW around the X-Axis */, 90 /* then rotate CCW around the Z-axis */, 0);
        OpenGLMatrix t = OpenGLMatrix.translation(  -mmFTCFieldWidth/2, /* send back to the RED beacon wall */
                                                    -mmFTCTileWidth/2, /* slide 1/2 tile toward the RED corner ramp */
                                                     mmPictureHieght/2 + mmFloorClearance /* Raise center of picture up */);
        toolsLocationMatrix = t.multiplied(r);
        RobotLog.ii(TAG, "Tools Beacon=%s", toolsLocationMatrix.formatAsTransform());

        // Location matrix for Gears RED Beacon
        r = Orientation.getRotationMatrix(
                AxesReference.EXTRINSIC, /* using the world (field) coordinate system) */
                AxesOrder.XZX,
                AngleUnit.DEGREES, 90 /* 1st rotate CCW around the X-Axis */, 90 /* then rotate CCW around the Z-axis */, 0);
        t = OpenGLMatrix.translation(  -mmFTCFieldWidth/2, /* send back to RED beacon the wall */
                1.5f * mmFTCTileWidth/2, /* slide 1 and 1/2 tile away from the RED corner ramp */
                mmPictureHieght/2 + mmFloorClearance /* Raise center of picture up */);
        gearsLocationMatrix = t.multiplied(r);
        RobotLog.ii(TAG, "Gears Beacon=%s", gearsLocationMatrix.formatAsTransform());

        // Location matrix for BLUE Wheels Beacon
        r = Orientation.getRotationMatrix(
                AxesReference.EXTRINSIC, /* using the world (field) coordinate system) */
                AxesOrder.XZX,
                AngleUnit.DEGREES, 0, 90 /* then rotate CCW around the Z-axis */, 0);
        t = OpenGLMatrix.translation(  mmFTCFieldWidth/2, /* send back to Blue beacon the wall */
                .5f * mmFTCTileWidth/2, /* slide 1/2 towards the Blue corner ramp */
                mmPictureHieght/2 + mmFloorClearance /* Raise center of picture up */);
        wheelsLocationMatrix = t.multiplied(r);
        RobotLog.ii(TAG, "Gears Beacon=%s", wheelsLocationMatrix.formatAsTransform());

        // Location matrix for BLUE Legos Beacon
        r = Orientation.getRotationMatrix(
                AxesReference.EXTRINSIC, /* using the world (field) coordinate system) */
                AxesOrder.XZX,
                AngleUnit.DEGREES, 0, 90 /* then rotate CCW around the Z-axis */, 0);
        t = OpenGLMatrix.translation(  mmFTCFieldWidth/2, /* send back to Blue beacon the wall */
                -1.5f * mmFTCTileWidth/2, /* slide 1 and 1/2 tile away from the Blue corner ramp */
                mmPictureHieght/2 + mmFloorClearance /* Raise center of picture up */);
        legosLocationMatrix = t.multiplied(r);
        RobotLog.ii(TAG, "Gears Beacon=%s", legosLocationMatrix.formatAsTransform());

        /**
         * Let the trackable listeners we care about know where the phone is. We know that each
         * listener is a {@link VuforiaTrackableDefaultListener} and can so safely cast because
         * we have not ourselves installed a listener of a different type.
         */
        OpenGLMatrix phoneLocationOnRobot = robot.getPhoneLocationOnRobot();
        RobotLog.ii(TAG, "phone=%s", phoneLocationOnRobot.formatAsTransform());

        ((VuforiaTrackableDefaultListener)toolsAsset.getListener()).setPhoneInformation(phoneLocationOnRobot, parameters.cameraDirection);
        ((VuforiaTrackableDefaultListener)gearsAsset.getListener()).setPhoneInformation(phoneLocationOnRobot, parameters.cameraDirection);

        telemetry.addData("Status", "Initialized");

    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
     */
    @Override
    public void init_loop() {
    }

    /*
     * Code to run ONCE when the driver hits PLAY
     */
    @Override
    public void start() {
        /** Start tracking the data sets we care about. */
        ftcAssets.activate();

        telemetry.addData("Status", "Started");

    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {
        int n = 2;

        for (VuforiaTrackable trackable : allTrackables) {
            OpenGLMatrix robotLocationTransform = ((VuforiaTrackableDefaultListener) trackable.getListener()).getUpdatedRobotLocation();
            if (robotLocationTransform != null) {
                lastLocation = robotLocationTransform;
            }
        }

        int newLeftTarget = robot.leftMotor.getCurrentPosition();
        int newRightTarget = robot.rightMotor.getCurrentPosition();

        telemetry.addData("state",  "%7d.%7d", majorStep,  minorStep);

        // Traverse a 1 foot square
        // major step 1:
        //  minor Step 1.1: start moving forward <n> foot.
        //  minor Step 1.2: wait until robot has finished moving forward 1 ft
        // major step 2: turn right
        //  minor step 2.1: start turning ... right motor forward , left motor backward
        //  minor step 2.2: wait until robot has finished turning, then repeat step 1 &2
        // major Step 3. after repeating step 1&2 4 times... stop and stay put
        switch (majorStep)
        {
            case 0: {

                majorStep = 1;
                minorStep = 1;
            }

            // major step #1: move forward 1 foot
            case 1: {
                // minor step #1.1: start moving forward
                if ( minorStep == 1) {
                    newLeftTarget  +=  (int) (n*12 * robot.ENCODER_COUNTS_PER_INCH);
                    newRightTarget +=  (int) (n*12 * robot.ENCODER_COUNTS_PER_INCH);

                    robot.leftMotor.setTargetPosition(newLeftTarget);
                    robot.rightMotor.setTargetPosition(newRightTarget);

                    robot.leftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    robot.rightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                    robot.leftMotor.setPower(0.6);
                    robot.rightMotor.setPower(0.6);

                     minorStep++;
                }
                // minor step #1.2: wait for robot to complete moving 1 ft forward
                else if (minorStep == 2) {
                        if (robot.leftMotor.isBusy() && robot.rightMotor.isBusy())
                            ; // do nothing motors are still working on getting to target
                        else {
                           // majorStep++;
                           // minorStep = 1;
                        }

                    }
            }  // completed major step #1: move forward 1 foot

            // major step #2: turn right
            case 2: {
                // minor step #2.1: start Turn to the right (dead reckoning turn)
                if ( minorStep == 1) {
                    newLeftTarget  +=  (int) (robot.ROBOT_WHEEL_BASE_INCHES/8 * robot.ENCODER_COUNTS_PER_INCH);
                    newRightTarget +=  (int) (robot.ROBOT_WHEEL_BASE_INCHES/8 * robot.ENCODER_COUNTS_PER_INCH);

                    robot.leftMotor.setTargetPosition(newLeftTarget);
                    robot.rightMotor.setTargetPosition(newRightTarget);

                    robot.leftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    robot.rightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                    robot.leftMotor.setPower(0.6);
                    robot.rightMotor.setPower(-0.6);

                    minorStep++;
                }
                // minor step #2.2: wait for robot to complete turn
                else if (minorStep == 2) {
                        if (robot.leftMotor.isBusy() && robot.rightMotor.isBusy())
                            ; // do nothing motors are still working on getting to target
                        else {
                            nTimes++;
                            if (nTimes == 4)
                            {
                                majorStep ++;
                            }
                            else {
                                majorStep = 1;  // repeat Step 1 & 2
                            }
                            minorStep = 1;
                        }
                    }
            } // completed major step #2: turn right

            // major step #3: done..
            case 3: {
                robot.leftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                robot.rightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

                robot.leftMotor.setPower(0);
                robot.rightMotor.setPower(-0);
            }  // completed major step #3: step
        }

        if (lastLocation != null) {
            //  RobotLog.vv(TAG, "robot=%s", format(lastLocation));
            telemetry.addData("Pos", lastLocation.formatAsTransform());
        } else {
            telemetry.addData("Pos", "Unknown");
        }
        telemetry.addData("Left", "Running to %7d", robot.leftMotor.getCurrentPosition());
        telemetry.addData("Right", "Running to %7d", robot.rightMotor.getCurrentPosition());

        telemetry.update();
    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
    }

}
