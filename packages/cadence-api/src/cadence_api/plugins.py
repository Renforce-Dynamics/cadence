from enum import Enum


class RootLossMode(str, Enum):
    NONE = "none"
    HOLD_LAST = "hold_last"
    SAFE_HALT = "safe_halt"
    FALLBACK = "fallback"
