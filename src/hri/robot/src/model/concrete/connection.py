import os

from furhat_remote_api import FurhatRemoteAPI

class RobotConnectionManager:
    #_FURHAT_IP = "localhost" 
    _FURHAT_IP = "143.225.85.144"
    _FURHAT_PORT = ""
    _session = None
    
    @staticmethod
    def get_session():
        """
        Returns a connection to furhat robot if session is not established yet, otherwise it will return the old session. 
        """
        if RobotConnectionManager._session is None:
            try:
                RobotConnectionManager._session = FurhatRemoteAPI(RobotConnectionManager._FURHAT_IP)
                print("Connection with Furhat successfully established!")
            except Exception as e:
                print("Unable to connect to Furhat:", e)
                os._exit(1)
                
        return RobotConnectionManager._session