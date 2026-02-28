package org.team100.lib.network;

public record SyncReply(long org, long rec, long xmt) {
    public static final SyncReplyStruct struct = new SyncReplyStruct();
}
