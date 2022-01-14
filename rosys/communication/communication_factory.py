from .serial_communication import SerialCommunication
from .web_communication import WebCommunication


class CommunicationFactory:

    @staticmethod
    def create():
        if SerialCommunication.is_possible():
            return SerialCommunication()
        if WebCommunication.is_possible():
            return WebCommunication()
