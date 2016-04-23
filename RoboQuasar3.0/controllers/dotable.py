class Dotable(dict):
    __getattr__ = dict.__getitem__

    def __init__(self, d):
        super(Dotable, self).__init__()
        self.update(**dict((k, self.parse(v))
                           for k, v in d.items()))

    @classmethod
    def parse(cls, v):
        if isinstance(v, dict):
            return cls(v)
        elif isinstance(v, list):
            return [cls.parse(i) for i in v]
        else:
            return v
