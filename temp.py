from enum import Enum

class Distance(Enum):
    CLOSE = 0
    MEDIUM = 1
    FAR = 2

print(Distance.CLOSE.value)