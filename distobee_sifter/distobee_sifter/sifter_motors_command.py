from dataclasses import dataclass
import json

@dataclass
class SifterMotorsCommand:
    brush_pwm: int | None
    vibration: bool | None

    def to_json(self) -> str:
        return json.dumps(self.__dict__)

    @staticmethod
    def from_json(data: str) -> 'SifterMotorsCommand':
        json_data = json.loads(data)
        ret = SifterMotorsCommand(**json_data)

        if ret.brush_pwm is not None:
            if not (-100 <= ret.brush_pwm <= 100):
                ret.brush_pwm = None
                print("Warning: brush_pwm out of range, set to None")

        return ret
