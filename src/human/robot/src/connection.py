from furhat_remote_api import FurhatRemoteAPI

class RobotConnectionManager:
    _FURHAT_IP = "127.0.0.1"
    _FURHAT_PORT = ""
    _session = None
    
    @staticmethod
    def get_session():
        """
        Returns a connection to nao robot if session is not established yet, otherwise it will return the old session. 
        """
        if RobotConnectionManager._session is None:
            try:
                RobotConnectionManager._session = FurhatRemoteAPI("localhost")
                print("Connection with Nao successfully established!")
            except Exception as e:
                print("Unable to connect to Furhat:", e)
                exit(1)
                
        return RobotConnectionManager._session