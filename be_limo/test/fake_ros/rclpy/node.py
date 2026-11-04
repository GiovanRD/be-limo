import collections

from builtin_interfaces.msg import Time
from rclpy.qos import DurabilityPolicy, QoSProfile


class Bus:
    def __init__(self):
        self.subs = collections.defaultdict(list)
        self.latched = {}
        self.now = 0.0
        self.log = []

    def publish(self, topic, msg, qos):
        if isinstance(qos, QoSProfile) and qos.durability == DurabilityPolicy.TRANSIENT_LOCAL:
            self.latched[topic] = msg
        for cb in list(self.subs[topic]):
            cb(msg)


BUS = Bus()


class _Param:
    def __init__(self, value):
        self.value = value


class _Logger:
    def __init__(self, name):
        self.name = name

    def _log(self, level, msg, **kw):
        BUS.log.append((BUS.now, self.name, level, msg))

    def info(self, msg, **kw):
        self._log('INFO', msg)

    def warn(self, msg, **kw):
        self._log('WARN', msg)

    warning = warn

    def error(self, msg, **kw):
        self._log('ERROR', msg)


class _Publisher:
    def __init__(self, topic, qos):
        self.topic, self.qos = topic, qos

    def publish(self, msg):
        BUS.publish(self.topic, msg, self.qos)

    def get_subscription_count(self):
        return len(BUS.subs[self.topic])


class _Timer:
    def __init__(self, period, cb):
        self.period, self.cb, self.next = period, cb, BUS.now + period
        self.timer_period_ns = int(period * 1e9)

    def cancel(self):
        self.period = None


class _Client:
    def __init__(self, name):
        self.name = name
        self.calls = []

    def service_is_ready(self):
        return False

    def call_async(self, req):
        self.calls.append(req)


class _Clock:
    def now(self):
        class _T:
            def to_msg(self_inner):
                return Time(sec=int(BUS.now), nanosec=int((BUS.now % 1) * 1e9))
        return _T()


class Node:
    ALL = []

    def __init__(self, name):
        self.name = name
        self._params = {}
        self.timers = []
        self._overrides = Node.OVERRIDES.get(name, {})
        Node.ALL.append(self)

    def declare_parameter(self, name, value=None):
        self._params[name] = _Param(self._overrides.get(name, value))
        return self._params[name]

    def get_parameter(self, name):
        return self._params[name]

    def get_logger(self):
        return _Logger(self.name)

    def get_clock(self):
        return _Clock()

    def create_publisher(self, msg_type, topic, qos):
        return _Publisher(topic, qos)

    def create_subscription(self, msg_type, topic, cb, qos):
        BUS.subs[topic].append(cb)
        if topic in BUS.latched:
            cb(BUS.latched[topic])

    def create_timer(self, period, cb, clock=None):
        t = _Timer(period, cb)
        self.timers.append(t)
        return t

    def destroy_timer(self, t):
        t.cancel()

    def create_client(self, srv_type, name):
        return _Client(name)

    def destroy_node(self):
        pass


Node.OVERRIDES = {}
