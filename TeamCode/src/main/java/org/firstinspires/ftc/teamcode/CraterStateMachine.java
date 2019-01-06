package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name="CraterStateMachine", group="VoyagerBot")
public class CraterStateMachine extends BaseStateMachineOpMode {

    final PathSeg[] wallPath1 = {
            new PathSeg(4, -4, 0.4),
            new PathSeg(-13, -13, 0.4),
            //new PathSeg (-2, -2, 0.4)

    };
    final PathSeg[] wallPath2 = {
            new PathSeg(-16, -16, 0.4)
    };
    final PathSeg[] wallPath3 = {
            new PathSeg(-4, 4, 0.4),
            new PathSeg(-13,-13,0.4)
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
