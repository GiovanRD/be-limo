class DurabilityPolicy:
    TRANSIENT_LOCAL = 'transient_local'
    VOLATILE = 'volatile'


class ReliabilityPolicy:
    RELIABLE = 'reliable'
    BEST_EFFORT = 'best_effort'


class QoSProfile:
    def __init__(self, depth=10, durability=DurabilityPolicy.VOLATILE, reliability=ReliabilityPolicy.RELIABLE):
        self.depth, self.durability, self.reliability = depth, durability, reliability
