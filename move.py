import time
import os
import json
from read_json import JamieControl

class Control(JamieControl):
    def __init__(self, 
                 arduino_port='COM8', 
                 baud_rate=115200,
                 joint_config_file='Joint_config.json',
                 behavior_folder='behaviors',
                 emote_file='Emote.json',
                 audio_folder='audio',
                 starting_voice='Matt'):
        JamieControl.__init__(self, arduino_port, baud_rate, joint_config_file, emote_file, audio_folder, starting_voice)
        with open(joint_config_file, 'r') as f:
            self.full_joint_config = json.load(f)['JointConfig']
        self.full_joint_map = {joint['JointName']: joint for joint in self.full_joint_config}
        self.behavior_folder = behavior_folder if os.path.exists(behavior_folder) else 'behaviors'
        self.initialize_serial_connection()

    def load_behavior(self, behavior_file):
        with open(behavior_file, 'r') as file:
            return json.load(file)['Keyframes']
        
    def delay(self, t):
        time.sleep(t)

    def move_to_home(self):
        joint_ids = [joint['JointID'] for joint in self.full_joint_config]
        home_angles = [joint['HomeAngle'] for joint in self.full_joint_config]
        self.send_joint_command(joint_ids, home_angles, 1)

    def perform_behavior(self, file):
        selected_behavior = file
        behavior_path = os.path.join(self.behavior_folder, selected_behavior)
        behavior_motion = self.load_behavior(behavior_path)
        for frame in behavior_motion:
            # Process Audio if available.
            if frame["HasAudio"] == "True":
                audio_clip = frame.get("AudioClip", frame.get("Expression", "default_audio"))
                self.audio_manager.send_audio(audio_clip)
            # Process Emote if available.
            if frame["HasEmote"] == "True":
                expression = frame.get("Expression", "Neutral")
                emote_value = self.emote_mapping.get(expression, 0)
                self.send_emote(emote_value)
            # Process Joint Commands if available.
            if frame["HasJoints"] == "True":
                joint_ids = [self.get_joint_id(j['Joint']) for j in frame['JointAngles']]
                angles = [j['Angle'] for j in frame['JointAngles']]
                move_time = frame["JointMoveTime"]
                self.send_joint_command(joint_ids, angles, move_time)
            self.delay(frame["WaitTime"] / 1000)
        self.move_to_home()
