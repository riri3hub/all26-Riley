package org.team100.lib.network;

public record SyncRequest(long org) {
    public static final SyncRequestStruct struct = new SyncRequestStruct();
}
