package frc.robot;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.networktables.StructSubscriber;
import edu.wpi.first.networktables.TimestampedObject;
import edu.wpi.first.wpilibj.RobotController;

/** The server end of the Sync prototcol. */
public class Sync implements Runnable {
    private final NetworkTableInstance inst;
    private final StructSubscriber<SyncRequest> sub;
    private final StructPublisher<SyncReply> pub;

    public Sync(NetworkTableInstance i) {
        inst = i;
        sub = inst.getStructTopic("syncrequest", SyncRequest.struct).subscribe(
                new SyncRequest(0));
        pub = inst.getStructTopic("syncreply", SyncReply.struct).publish();
    }

    @Override
    public void run() {
        // Reply if a message is waiting.
        TimestampedObject<SyncRequest>[] queue = sub.readQueue();
        int n = queue.length;
        if (n > 0) {
            // reply to the most recent, ignore stale entries
            long org = queue[n - 1].value.org();
            long now = RobotController.getFPGATime();
            pub.set(new SyncReply(org, now, now));
            inst.flush();
        }
    }
}
