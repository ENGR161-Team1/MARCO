from buildhat import Motor, MotorPair

class Motion:
  def __init__(self, **kwargs):
    motors = MotorPair(kwargs["motors"][0], kwargs["motors"][1])
