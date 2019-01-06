package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

@Autonomous(name="BestCraterOption", group="VoyagerBot")
@Disabled
public class RedCrater extends StateMachineExample {
    @Override
    public void init() {
        super.init();
        robot.teamMarker.setPosition(0);
    }

    final PathSeg[] wallPath11 = {
            new PathSeg(-13, -13, 0.4),
            new PathSeg(-11, 11, 0.4),
            new PathSeg(-3,-3,0.4)
            //new PathSeg (-2, -2, 0.4)

    };

    final PathSeg[] wallPath1 = {
            new PathSeg(-18, -18, 0.4)
    };

    final PathSeg[] wallPath2 = {
            new PathSeg(-18, -18, 0.4)
    };

    final PathSeg[] wallPath3 = {
            new PathSeg(-18, -18, 0.4)
    };

    final PathSeg[] wallPath31 = {
            new PathSeg(4, -4, 0.4),
            new PathSeg(-10,-10,0.4)
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
