package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name="DepotStateMachine", group="VoyagerBot")
public class DepotStateMachine extends BaseStateMachineOpMode {

    final PathSeg[] wallPath1 = {
            new PathSeg(4, -4, 0.4),
            new PathSeg(-20, -20, 0.4),
            new PathSeg(4, -4, 0.4),
            new PathSeg(-8, -8, 0.4),
            //new PathSeg (-2, -2, 0.4)

    };
    final PathSeg[] wallPath2 = {
            new PathSeg(-22, -22, 0.4)
    };
    final PathSeg[] wallPath3 = {
            new PathSeg(-6, 6, 0.4),
            new PathSeg(-20,-20,0.4)
    };

    @Override
    PathSeg[] getWallPath() {
        if (goldPosition == StateMachineExample.GoldPosition.ONE) {
            return(wallPath1);
        } else if (goldPosition == StateMachineExample.GoldPosition.TWO) {
            return(wallPath2);
        } else {
            return(wallPath3);
        }
    };
}
