package frc.robot;

public record SyncRequest(long org) {
    public static final SyncRequestStruct struct = new SyncRequestStruct();
}
