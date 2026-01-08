package pedroPathing.constants;

import com.pedropathing.localization.GoBildaPinpointDriver;
import com.pedropathing.localization.constants.PinpointConstants;
import com.pedropathing.localization.localizers.DriveEncoderLocalizer;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class LConstants30630 {
    static {
        DriveEncoderLocalizer.TURN_TICKS_TO_RADIANS = 6.4789;
        DriveEncoderLocalizer.FORWARD_TICKS_TO_INCHES = 47.4292;
        DriveEncoderLocalizer.STRAFE_TICKS_TO_INCHES = 47.1165;
        PinpointConstants.forwardY = -7.8;
        PinpointConstants.strafeX = 7.5;
        PinpointConstants.distanceUnit = DistanceUnit.INCH;
        PinpointConstants.hardwareMapName = "pinpoint";
        PinpointConstants.useYawScalar = false;
        PinpointConstants.yawScalar = 1.0;
        PinpointConstants.useCustomEncoderResolution = false;
        PinpointConstants.encoderResolution = GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_SWINGARM_POD;
        PinpointConstants.customEncoderResolution = 19.89436789;//this is only used if using custom deadwheels
        PinpointConstants.forwardEncoderDirection = GoBildaPinpointDriver.EncoderDirection.REVERSED;
        PinpointConstants.strafeEncoderDirection = GoBildaPinpointDriver.EncoderDirection.FORWARD;
    }
}




