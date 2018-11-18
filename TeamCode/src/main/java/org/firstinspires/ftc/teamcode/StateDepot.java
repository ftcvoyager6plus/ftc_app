package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name="Depot", group="VoyagerBot")
public class StateDepot extends StateMachineExample  {
    @Override
    public void init() {
        super.init();
        robot.teamMarker.setPosition(0);
    }

    final PathSeg[] wallPath1 = {
            new PathSeg(-15, -15, 0.4),
            new PathSeg(-10, 10, 0.4),
            new PathSeg(-22, -22, 0.4)
    };
    final PathSeg[] wallPath2 = {
            new PathSeg(-30, -30, 0.4)
    };
    final PathSeg[] wallPath3 = {
            new PathSeg(-12, -12, 0.4),
            new PathSeg(10, -10, 0.4),
            new PathSeg(-22, -22, 0.4)
    };

    @Override
    PathSeg[] getWallPath() {
        if (goldPosition == GoldPosition.ONE) {
            return(wallPath1);
        } else if (goldPosition == GoldPosition.TWO) {
            return(wallPath2);
        } else {
            return(wallPath3);
        }
    };
}
