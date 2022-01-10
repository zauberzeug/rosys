from .serial_communication import SerialCommunication
from .web_communication import WebCommunication


class NoCommunicationError(Exception):
    pass


class CommunicationFactory:

    @staticmethod
    def create():
        if SerialCommunication.is_possible():
            return SerialCommunication()
        elif WebCommunication.is_possible():
            return WebCommunication()
        else:
            raise NoCommunicationError('no communication possible')
