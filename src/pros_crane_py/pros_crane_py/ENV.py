import pydantic
from typing import List

CRANE_STATE = "crane_state"
CRANE_CONTROL = "crane"

class CraneMovementControl(pydantic.BaseModel):    
    moveX: float = 0
    moveY: float = 0
    moveZ: float = 0