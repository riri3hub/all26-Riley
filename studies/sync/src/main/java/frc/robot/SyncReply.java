package frc.robot;

public record SyncReply(long org, long rec, long xmt) {
    public static final SyncReplyStruct struct = new SyncReplyStruct();
}
