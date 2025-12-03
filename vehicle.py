from pymavlink import mavutil
class vehicle:
    connection_string =""
    baudrate = 0
    #master =0 #mavutil.mavlink_connection turunde olmali
    last_heartbeat= None
    is_connected = False
    is_armed = False
    mode = ""
    position={
        "lat": float,
        "lon":float,
        "alt": float
    }
    velocity = {
    "vx": float,   
    "vy": float,   
    "vz": float    
}
    attitude={
            "roll": float,  
            "pitch": float, 
            "yaw": float    
        }
    battery={
            "voltage": float, 
            "current": float, 
            "level": float   
        }
    system_status=""
    vehicle_type=""
    mission=[]
    last_message_time=0.0
    home_location={
        "lat": None,
        "lon": None,
        "alt": None,
    }
    flight_start_time=0.0
    firmware_version=""
    satellites_visible=0
    gps_fix_type=0
    last_command_ack={
        "command": None,
        "result": None,
        "progress": None,
        "param2": None,
        "target_system": None,
        "target_component": None,
        "timestamp": None
    }
    massage_buffer=[]
    
    def __init__(self, connection_string, baudrate):
        self.connection_string = connection_string
        self.baudrate = baudrate