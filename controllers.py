"""
PID Controller

components:
    follow attitude commands
    gps commands and yaw
    waypoint following
"""
import numpy as np
import time

DRONE_MASS_KG = 2
GRAVITY = -9.81


class PDController(object):

    def __init__(
        self,
        max_speed=15,
        turn_speed=2,
        max_tilt=0.5,
        max_ascent_rate=3,
        max_descent_rate=1,
        Kp_hdot=10,
        Kp_yaw=6.5,
        Kp_r=20,
        Kp_roll=6.5,
        Kp_q=20,
        Kp_pitch=6.5,
        Kp_p=20,
        Kp_pos=0.10,
        Kp_vel=0.3,
        Kd_vel=0,
        Kp_alt=0.4,
        Ki_hdot=0.1
    ):
        self.max_speed = max_speed
        self.turn_speed = turn_speed
        self.max_tilt = max_tilt
        self.max_ascent_rate = max_ascent_rate
        self.max_descent_rate = max_descent_rate
        self.Kp_hdot = Kp_hdot
        self.Kp_yaw = Kp_yaw
        self.Kp_r = Kp_r
        self.Kp_roll = Kp_roll
        self.Kp_p = Kp_p
        self.Kp_pitch = Kp_pitch
        self.Kp_q = Kp_q
        self.Kp_pos = Kp_pos
        self.Kp_vel = Kp_vel
        self.Kd_vel = Kd_vel
        self.Kp_alt = Kp_alt
        self.Ki_hdot = Ki_hdot
        self.last_vel_error_body = np.float32([0, 0, 0])
        self.time_since_last_update = time.time()
        self.hdot_int = 0


    def velocity_control(self, target_velocity, attitude, angular_velocity, local_velocity):
        """
        target_velocity: 3-element numpy array (NED frame), target local velocity
        attitude: 3-element numpy array (roll,pitch,yaw) in radians
        angular_velocity: 3-element array  (roll_rate, pitch_rate, yaw_rate) in radians/sec
        local_velocity: 3-element array (NED frame) in meters/sec, current velocity
        """
        
        target_speed = np.linalg.norm(target_velocity[0:2])
        if(target_speed>self.max_speed):
            target_velocity[0] = self.max_speed*target_velocity[0]/target_speed
            target_velocity[1] = self.max_speed*target_velocity[1]/target_speed
        
        cos_yaw = np.cos(attitude[2])
        sin_yaw = np.sin(attitude[2])

        vel_error = target_velocity-local_velocity
        vel_error_body = np.float32([0, 0, 0])
        vel_error_body[0] = cos_yaw * vel_error[0] + sin_yaw * vel_error[1]
        vel_error_body[1] = -sin_yaw * vel_error[0] + cos_yaw * vel_error[1]
        
        
        pitch_cmd = -self.Kp_vel * vel_error_body[0]
        roll_cmd = self.Kp_vel * vel_error_body[1]

        attitude_cmd = np.array([roll_cmd,pitch_cmd,0.0])
            
        roll_moment,pitch_moment = self.roll_pitch_control(attitude_cmd, attitude, angular_velocity)
        thrust = self.vertical_velocity_control(target_velocity[2],attitude,local_velocity[2])
            
        return thrust, roll_moment, pitch_moment


    def position_control(self, target_position, attitude, angular_velocity, local_velocity, local_position):
        """
        target_position: 3-element numpy array (NED frame), target position
        attitude: 3-element numpy array (roll,pitch,yaw) in radians
        angular_velocity: 3-element array  (roll_rate, pitch_rate, yaw_rate) in radians/sec
        local_velocity: 3-element array (NED frame) in meters/sec, current velocity
        local_position: 3-element numpy array (NED frame), in meters, current position relative to home_position
        """
        
        
        position_error = target_position - local_position
        # print('local position', local_position)
        # print('target position', target_position)
        # print('position error', position_err)
        # print('euler angles', np.degrees(euler_angles))

        #Commanded velocity in the local NED frame
        velocity_cmd = np.float32([0, 0, 0])
        
        # deadband position error
        if np.linalg.norm([position_error[0], position_error[1]]) >= 1:
            velocity_cmd[0] = self.Kp_pos * position_error[0]
            velocity_cmd[1] = self.Kp_pos * position_error[1]
            
        velocity_cmd[2] = self.Kp_alt*position_error[2]
            
        return self.velocity_control(velocity_cmd, attitude, angular_velocity, local_velocity)
        
    
    """Yaw Controllers
    yaw_control controls the vehicle heading in order to point the front of vehicle in a specific direction
    yawrate_control stabilizes the vehicles heading rate or to set a desired yawrate
    """
    def yaw_control(self,target_yaw,yaw,yawrate):
        """
        target_yaw: desired vehicle yaw in radians
        yaw: vehicle yaw in radians
        yawrate: vehicle yawrate in radians/sec
        """
        
        #Ensure the target is within range of 0 to 2*pi
        target_yaw = np.mod(target_yaw,2.0*np.pi)
        
        yaw_error = target_yaw-yaw
        if(yaw_error > np.pi):
            yaw_error = yaw_error-2.0*np.pi
        elif(yaw_error<-np.pi):
            yaw_error = yaw_error+2.0*np.pi
        
        yawrate_cmd = self.Kp_yaw*yaw_error
        return self.yawrate_control(yawrate_cmd,yawrate)
    
    
    
    
    """The following three controllers are the inner most controllers. 
    They ae essentially stabilizing controllers
    I think these should be fixed within the simunlator (Unity) with the ability to set PID gains"""
    def yawrate_control(self,target_yawrate,yawrate):
        """
        target_yawrate: desired vehile yawrate in radians/sec
        yawrate: vehicle yawrate in radians/sec
        """
        yaw_moment = self.Kp_r*(target_yawrate-yawrate)
        return yaw_moment
    
    """Vertical Velocity Controller"""
    def vertical_velocity_control(self,target_vertical_velocity, attitude, vertical_velocity):
        """
        target_vertical_velocity: desired vertical velocity in meters/sec
        attitude: 3-element numpy array (roll,pitch,yaw) in radians
        local_velocity: 3-element array (NED frame) in meters/sec, current velocity
        """
        
        now = time.time()
        self.dt = now - self.time_since_last_update
        self.time_since_last_update = now
        thrust_nom = -DRONE_MASS_KG * GRAVITY
        hdot_error = 0
        if target_vertical_velocity > 0:
            hdot_error = self.max_ascent_rate * target_vertical_velocity - vertical_velocity
        else:
            hdot_error = self.max_descent_rate * target_vertical_velocity - vertical_velocity
        self.hdot_int += hdot_error * self.dt
        # self.hdot_int += 0

        # hdot to thrust
        thrust = (self.Kp_hdot * hdot_error + self.Ki_hdot * self.hdot_int + thrust_nom) / (np.cos(attitude[0]) * np.cos(attitude[1]))
        return thrust
    
    """Attitude Controllers """
    def roll_pitch_control(self, target_attitude, attitude, angular_rate):
        """
        target_attitude: 3-element numpy array (roll,pitch,yaw) in radians
        attitude: 3-element numpy array (roll,pitch,yaw) in radians
        angular_velocity: 3-element array  (roll_rate, pitch_rate, yaw_rate) in radians/sec
        """
        
        angle_magnitude = np.linalg.norm([target_attitude[0], target_attitude[1]])
        if angle_magnitude > self.max_tilt:
            target_attitude[1] = self.max_tilt * target_attitude[1] / angle_magnitude
            target_attitude[0] = self.max_tilt * target_attitude[0] / angle_magnitude
        
        #Roll controller
        roll = attitude[0]
        rollrate = angular_rate[0]
        target_roll = target_attitude[0]
        target_roll = np.mod(target_roll,2.0*np.pi)
        
        # angle to angular rate command (for pitch and roll)
        roll_error = target_roll - roll
        
        
        if(roll_error>np.pi):
            roll_error = roll_error-2.0*np.pi
        elif(roll_error<-np.pi):
            roll_error = roll_error+2.0*np.pi
        
        roll_rate_error = self.Kp_roll * roll_error - rollrate

        # angular rate to moment (pitch and roll)
        roll_moment = self.Kp_p * roll_rate_error
        
        
        pitch = attitude[1]
        pitchrate = angular_rate[1]
        target_pitch = target_attitude[1]
        target_pitch = np.mod(target_pitch,2.0*np.pi)
        
        # angle to angular rate command (for pitch and roll)
        pitch_error = target_pitch - pitch
        
        
        if(pitch_error>np.pi):
            pitch_error = pitch_error-2.0*np.pi
        elif(pitch_error<-np.pi):
            pitch_error = pitch_error+2.0*np.pi
        
        pitch_rate_error = self.Kp_pitch * pitch_error - pitchrate

        # angular rate to moment (pitch and roll)
        pitch_moment = self.Kp_q * pitch_rate_error        

        return roll_moment, pitch_moment

