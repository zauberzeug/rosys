import abc
import logging


class Actor(abc.ABC):

    def __init__(self) -> None:
        self.name = __name__[:-5] + self.__class__.__name__
        self.log = logging.getLogger(self.name)
