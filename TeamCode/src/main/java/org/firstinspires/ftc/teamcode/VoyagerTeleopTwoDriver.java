package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name="V1TelopTwoDriver", group="VoyagerBot")
public class VoyagerTeleopTwoDriver extends VoyagerTelopOneDriver {

    @Override
    public boolean twoDriverMode() {
        return true;
    }
}
