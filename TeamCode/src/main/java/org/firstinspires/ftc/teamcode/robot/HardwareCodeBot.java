package org.firstinspires.ftc.teamcode.robot;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;


/**
 * Created by programmer on 10/3/2016.
 */

public class HardwareCodeBot {
    static final double     COUNTS_PER_MOTOR_REV    = 280 ;    // eg: andymark never rest motor 40
    static final double     DRIVE_GEAR_REDUCTION    = 1.0 ;     // This is < 1.0 if geared UP
    public static final double     WHEEL_DIAMETER_INCHES   = 4.0 ;     // For figuring circumference
    static final double     WHEEL_CIRCUMFRENCE_INCHES = WHEEL_DIAMETER_INCHES * 3.1415;
    public static final double     ROBOT_WHEEL_BASE_INCHES = 12;
    public static final double     ENCODER_COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_CIRCUMFRENCE_INCHES);
    public ModernRoboticsI2cRangeSensor rangeSensorFront;

    public DcMotor leftMotor   = null;
    public DcMotor  rightMotor  = null;

    public final float WIDTH_INCHES = 18;

    HardwareMap hwMap           =  null;
    private ElapsedTime period  = new ElapsedTime();

    public void init(HardwareMap ahwMap) {
        // Save reference to Hardware map
        hwMap = ahwMap;

        rangeSensorFront = ahwMap.get(ModernRoboticsI2cRangeSensor.class, "FrontRange");

        // Define and Initialize Motors
        leftMotor   = hwMap.dcMotor.get("Left");
        rightMotor  = hwMap.dcMotor.get("Right");
        rightMotor.setDirection(DcMotor.Direction.REVERSE); // Set to REVERSE if using AndyMark motors
        leftMotor.setDirection(DcMotor.Direction.FORWARD);// Set to FORWARD if using AndyMark motors

        // Set all motors to zero power
        leftMotor.setPower(0);
        rightMotor.setPower(0);

        leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void move(double lx, double rx, double px, double py) {
        // No point in moving if still busy
        if (this.leftMotor.isBusy() || this.rightMotor.isBusy()) {
            return;
        }

        this.leftMotor.setTargetPosition((int)lx);
        this.rightMotor.setTargetPosition((int)rx);

        if (this.leftMotor.getMode() != DcMotor.RunMode.RUN_TO_POSITION) {
            this.leftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }

        if (this.rightMotor.getMode() != DcMotor.RunMode.RUN_TO_POSITION) {
            this.rightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }

        this.leftMotor.setPower(px);
        this.rightMotor.setPower(py);
    }
    /***
     *
     * waitForTick implements a periodic delay. However, this acts like a metronome with a regular
     * periodic tick.  This is used to compensate for varying processing times for each cycle.
     * The function looks at the elapsed cycle time, and sleeps for the remaining time interval.
     *
     * @param periodMs  Length of wait cycle in mSec.
     * @throws InterruptedException
     */
    public void waitForTick(long periodMs) throws InterruptedException {

        long  remaining = periodMs - (long)period.milliseconds();

        // sleep for the remaining portion of the regular cycle period.
        if (remaining > 0)
            Thread.sleep(remaining);

        // Reset the cycle clock for the next pass.
        period.reset();
    }


    public OpenGLMatrix getPhoneLocationOnRobot()
    {
        /**
         * Create a transformation matrix describing where the phone is on the robot. Here, we
         * put the phone on the center of the robot with the screen facing in (see our
         * choice of BACK camera above) and in landscape mode. Starting from alignment between the
         * robot's and phone's axes, this is a rotation of -90deg along the Y axis.
         *
         * When determining whether a rotation is positive or negative, consider yourself as looking
         * down the (positive) axis of rotation from the positive towards the origin. Positive rotations
         * are then CCW, and negative rotations CW. An example: consider looking down the positive Z
         * axis towards the origin. A positive rotation about Z (ie: a rotation parallel to the the X-Y
         * plane) is then CCW, as one would normally expect from the usual classic 2D geometry.
         */
     OpenGLMatrix phoneLocationOnRobot = OpenGLMatrix
            .translation(0,0,0)
            .multiplied(Orientation.getRotationMatrix(
                    AxesReference.EXTRINSIC, AxesOrder.YZY,
                    AngleUnit.DEGREES, -90, 0, 0));
     return phoneLocationOnRobot;
    }
}
