package pedroPathing.teleops;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
@Disabled
@TeleOp(name = "BlueTeleop.j")
public class BlueTeleop extends JavaCompetitionTeleop{

    @Override
    protected AllianceColor getAllianceColor() {
       return AllianceColor.BLUE;
    }
}
