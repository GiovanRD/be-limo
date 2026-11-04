"""Minimal ROS message stand-ins for offline tests."""
import copy


def message(type_name, **fields):
    def __init__(self, **kw):
        for k, default in fields.items():
            setattr(self, k, kw.pop(k) if k in kw else (default() if callable(default) else copy.copy(default)))
        if kw:
            raise TypeError("%s has no field(s) %s" % (type_name, list(kw)))

    def __repr__(self):
        return "%s(%s)" % (type_name, ', '.join('%s=%r' % (k, getattr(self, k)) for k in fields))
    return type(type_name, (), {'__init__': __init__, '__repr__': __repr__, '_fields': fields})
