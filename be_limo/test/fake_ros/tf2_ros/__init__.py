class TransformBroadcaster:
    def __init__(self, node):
        self.sent = []

    def sendTransform(self, t):
        self.sent = [t]
