class _Stub(object):
    def __init__(self, *a, **k):
        pass

    def __getattr__(self, n):
        return _Stub()

    def __call__(self, *a, **k):
        return _Stub()


def __getattr__(name):
    return _Stub()
