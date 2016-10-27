package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;

import java.text.SimpleDateFormat;
import java.util.ArrayList;
import java.util.Date;
import java.util.List;

@Autonomous(name="Vuforia Test", group="Autonomous")
public class VuforiaTest extends OpMode {
    public static String VUFORIA_KEY = "AeRW4MT/////AAAAGU3Wbz2I0kTQr15Yq15bnjZ4YrOv67kvhALKU6MH0u3z0ZyRUtRYxew6uBX5fpNuw9kYDqpGQsf/nDdlqo6DQqAhsWt+gXRvJkAeOL3bQktk+qf8ZppR9l7d26VmwjcGTqFvHoB/zWAnbakjGE8TbXDTT85bE5HvR1Jy8pvstALmW4ikG1hDsQ1x+KQHPVMsEaR6iqKxQLDazzKI6+FJDtVurJw8TSbhGB6ojJkG6LJLiYIusbscWtMvxC9tBIioB7O7DxaGSTQP2EVqJF/FwfgqAaXOuxxC+1chTAwCnq0l2p/DJEU0U4guo5iMkyfmkwZDeZ/2RIFVAHFUycyf6n8IQymSW+Aej5v+86IDlx0n";

    public VuforiaLocalizer vuforia;
    public List<VuforiaTrackable> allTrackables = new ArrayList<VuforiaTrackable>();

    String format(OpenGLMatrix transformationMatrix) {
        return transformationMatrix.formatAsTransform();
    }

    public void init() {
        String TAG = "Vuforia 11588";
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.
                Parameters(com.qualcomm.ftcrobotcontroller.R.id.cameraMonitorViewId);
        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;
        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        this.vuforia = ClassFactory.createVuforiaLocalizer(parameters);

        VuforiaTrackables ftcAssets = this.vuforia.loadTrackablesFromAsset("FTC_2016-17");
        VuforiaTrackable wheelsAsset = ftcAssets.get(0);
        wheelsAsset.setName("Wheels");
        VuforiaTrackable toolsAsset = ftcAssets.get(1);
        toolsAsset.setName("Tools");
        VuforiaTrackable legosAsset = ftcAssets.get(2);
        legosAsset.setName("Legos");
        VuforiaTrackable gearsAsset = ftcAssets.get(3);
        gearsAsset.setName("Gears");

        allTrackables = new ArrayList<VuforiaTrackable>();
        allTrackables.addAll(ftcAssets);

        float mmPerInch        = 25.4f;
        float mmBotWidth       = 5 * mmPerInch;
        float mmFTCFieldWidth  = (12*12 - 2) * mmPerInch;   // the FTC field is ~11'10" center-to-center of the glass panels
        float mmBoxWidth       = mmFTCFieldWidth / 6;
        float mmPicWidth       = 11 * mmPerInch;

        // Gear location
        OpenGLMatrix gearLocation = OpenGLMatrix
                .translation(-mmFTCFieldWidth / 2, -(mmBoxWidth + mmPicWidth) / 2, 0)
                .multiplied(Orientation.getRotationMatrix(
                        AxesReference.EXTRINSIC, AxesOrder.XZX,
                        AngleUnit.DEGREES, 90, 90, 0));
        gearsAsset.setLocation(gearLocation);

        RobotLog.ii(TAG, "Gears=%s", format(gearLocation));

        // Tool location

        OpenGLMatrix toolsLocation = OpenGLMatrix
                .translation(-mmFTCFieldWidth / 2, mmBoxWidth * 1.5f - (mmPicWidth / 2), 0)
                .multiplied(Orientation.getRotationMatrix(
                        AxesReference.EXTRINSIC, AxesOrder.XZX,
                        AngleUnit.DEGREES, 90, 90, 0));
        toolsAsset.setLocation(toolsLocation);

        RobotLog.ii(TAG, "Tools=%s", format(toolsLocation));

        // Legos location

        OpenGLMatrix legosLocation = OpenGLMatrix
                .translation(-mmBoxWidth * 1.5f - (mmPicWidth / 2), mmFTCFieldWidth / 2, 0)
                .multiplied(Orientation.getRotationMatrix(
                        AxesReference.EXTRINSIC, AxesOrder.XZX,
                        AngleUnit.DEGREES, 90, 0, 0));
        legosAsset.setLocation(legosLocation);

        RobotLog.ii(TAG, "Legos=%s", format(legosLocation));

        // Wheels location

        OpenGLMatrix wheelsLocation = OpenGLMatrix
                .translation((mmBoxWidth - mmPicWidth) / 2, mmFTCFieldWidth / 2, 0)
                .multiplied(Orientation.getRotationMatrix(
                        AxesReference.EXTRINSIC, AxesOrder.XZX,
                        AngleUnit.DEGREES, 90, 0, 0));
        wheelsAsset.setLocation(wheelsLocation);

        RobotLog.ii(TAG, "Wheels=%s", format(wheelsLocation));

        // Phone location on robot

        OpenGLMatrix phoneLocationOnRobot = OpenGLMatrix
                .translation(0,0,0)
                .multiplied(Orientation.getRotationMatrix(
                        AxesReference.EXTRINSIC, AxesOrder.YZY,
                        AngleUnit.DEGREES, -90, 0, 0));
        RobotLog.ii(TAG, "phone=%s", format(phoneLocationOnRobot));

        ((VuforiaTrackableDefaultListener)wheelsAsset.getListener())
                .setPhoneInformation(phoneLocationOnRobot, parameters.cameraDirection);
        ((VuforiaTrackableDefaultListener)legosAsset.getListener())
                .setPhoneInformation(phoneLocationOnRobot, parameters.cameraDirection);
        ((VuforiaTrackableDefaultListener)toolsAsset.getListener())
                .setPhoneInformation(phoneLocationOnRobot, parameters.cameraDirection);
        ((VuforiaTrackableDefaultListener)gearsAsset.getListener())
                .setPhoneInformation(phoneLocationOnRobot, parameters.cameraDirection);

         ftcAssets.activate();
    }

    public void loop() {
        for (VuforiaTrackable trackable : allTrackables) {
            OpenGLMatrix robotLocation = ((VuforiaTrackableDefaultListener)trackable.getListener())
                                                .getUpdatedRobotLocation();

            if (robotLocation != null) {
                telemetry.addData(trackable.getName(), " Is Visible");
            }
        }

        telemetry.update();
    }
}