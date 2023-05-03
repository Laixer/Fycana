import numpy as np

def numpy3d_to_string(array):
    """Convert a numpy array to a string"""
    if array is None:
        return "None"
    return f"[{array[0]:.2f}, {array[1]:.2f}, {array[2]:.2f}]"


class MockEncoder:
    def __init__(self, id, resolution, lower_bound, upper_bound, initial_value=None):
        self.id = id
        self.resolution = resolution
        self.lower_bound = lower_bound
        self.upper_bound = upper_bound
        if initial_value is None:
            if self.lower_bound == -np.inf:
                self._value = 0
            else:
                self._value = self.lower_bound
        else:
            self._value = initial_value

    @property
    def value(self) -> int:
        """Get the value of the encoder"""
        return int(self._value)

    @value.setter
    def value(self, value: float):
        """Set the value of the encoder"""
        self._value = value * self.resolution

    @classmethod
    def random(cls, id, resolution, lower_bound, upper_bound):
        """Get a random value between the bounds"""
        initial_value = np.round(np.random.uniform(lower_bound, upper_bound) * resolution, 0)

        encoder = cls(id, resolution, lower_bound, upper_bound, initial_value)
        return encoder

    def decode(self) -> float:
        """Decode the value of the encoder"""
        return self._value / self.resolution

    def increase_by_rad(self, rad: float) -> float:
        """Increase the value by a given amount of radians"""
        self._value += rad * self.resolution
        return self.value

    def decrease_by_rad(self, rad: float) -> float:
        """Decrease the value by a given amount of radians"""
        self._value -= rad * self.resolution
        return self.value