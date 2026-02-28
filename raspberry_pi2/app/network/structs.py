import dataclasses
from wpiutil import wpistruct


@wpistruct.make_wpistruct  # type:ignore
@dataclasses.dataclass
class SyncRequest:
    org: wpistruct.int64


@wpistruct.make_wpistruct  # type:ignore
@dataclasses.dataclass
class SyncReply:
    org: wpistruct.int64
    rec: wpistruct.int64
    xmt: wpistruct.int64
