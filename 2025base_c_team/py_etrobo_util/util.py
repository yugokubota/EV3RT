
class SymmetricClamper:
    def __init__(self, min_val: float, max_val: float):
        assert 0 <= min_val <= max_val, "Require 0 <= min_val <= max_val"
        self.min_val = min_val
        self.max_val = max_val

    def clamp(self, value: float) -> float:
        if value > 0:
            return max(self.min_val, min(value, self.max_val))
        elif value < 0:
            return min(-self.min_val, max(value, -self.max_val))
        else:
            return 0.0