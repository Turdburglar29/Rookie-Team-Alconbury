package pedroPathing.constants;

import com.pedropathing.localization.GoBildaPinpointDriver;
import com.pedropathing.localization.constants.PinpointConstants;
import com.pedropathing.localization.localizers.DriveEncoderLocalizer;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class LConstants30630 {
    static {
        DriveEncoderLocalizer.TURN_TICKS_TO_RADIANS = 6.5184;
        DriveEncoderLocalizer.FORWARD_TICKS_TO_INCHES = 46.8258;
        DriveEncoderLocalizer.STRAFE_TICKS_TO_INCHES = 46.9095;
        PinpointConstants.forwardY = -4;
        PinpointConstants.strafeX = -5.6;
        PinpointConstants.distanceUnit = DistanceUnit.INCH;
        PinpointConstants.hardwareMapName = "pinpoint";
        PinpointConstants.useYawScalar = false;
        PinpointConstants.yawScalar = 1.0;
        PinpointConstants.useCustomEncoderResolution = false;
        PinpointConstants.encoderResolution = GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_SWINGARM_POD;
        PinpointConstants.customEncoderResolution = 19.89436789;//this is only used if using custom deadwheels
        PinpointConstants.forwardEncoderDirection = GoBildaPinpointDriver.EncoderDirection.FORWARD;
        PinpointConstants.strafeEncoderDirection = GoBildaPinpointDriver.EncoderDirection.REVERSED;
    }
}




