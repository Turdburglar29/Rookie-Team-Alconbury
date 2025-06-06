package pedroPathing.teleops;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
@Disabled
@TeleOp(name = "RedTeleop.j")
public class RedTeleop extends JavaCompetitionTeleop{
    @Override
    protected AllianceColor getAllianceColor() {
        return AllianceColor.RED;
    }
}
