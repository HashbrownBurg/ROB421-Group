import time
import os
import json
from read_json import JamieControl

class JamieControlCLI(JamieControl):
    def __init__(self, 
                 arduino_port='COM8', 
                 baud_rate=115200,
                 joint_config_file='Joint_config.json',
                 behavior_folder='behaviors',
                 emote_file='Emote.json',
                 audio_folder='audio',
                 starting_voice='Matt'):
        super().__init__(arduino_port, baud_rate, joint_config_file, emote_file, audio_folder, starting_voice)
        
        with open(joint_config_file, 'r') as f:
            self.full_joint_config = json.load(f)['JointConfig']
        
        self.full_joint_map = {joint['JointName']: joint for joint in self.full_joint_config}
        self.behavior_folder = behavior_folder if os.path.exists(behavior_folder) else 'behaviors'
        self.initialize_serial_connection()

    def delay(self, t):
        time.sleep(t)

    def load_behavior(self, behavior_file):
        with open(behavior_file, 'r') as file:
            return json.load(file)['Keyframes']

    def move_to_home(self):
        joint_ids = [joint['JointID'] for joint in self.full_joint_config]
        home_angles = [joint['HomeAngle'] for joint in self.full_joint_config]
        self.send_joint_command(joint_ids, home_angles, 1)

    def get_behavior_files(self):
        return [f for f in os.listdir(self.behavior_folder) if f.endswith('.json')]

    def perform_behavior(self, behavior_name):
        behavior_path = os.path.join(self.behavior_folder, behavior_name)
        behavior_motion = self.load_behavior(behavior_path)
        for frame in behavior_motion:
            if frame.get("HasAudio") == "True":
                audio_clip = frame.get("AudioClip", frame.get("Expression", "default_audio"))
                self.audio_manager.send_audio(audio_clip)
            if frame.get("HasEmote") == "True":
                expression = frame.get("Expression", "Neutral")
                emote_value = self.emote_mapping.get(expression, 0)
                self.send_emote(emote_value)
            if frame.get("HasJoints") == "True":
                joint_ids = [self.get_joint_id(j['Joint']) for j in frame['JointAngles']]
                angles = [j['Angle'] for j in frame['JointAngles']]
                move_time = frame["JointMoveTime"]
                self.send_joint_command(joint_ids, angles, move_time)
            self.delay(frame["WaitTime"] / 1000)

    def close(self):
        self.close_connection()

def main():
    controller = JamieControlCLI(audio_folder="audio", starting_voice="Matt")
    
    # Example: move to home
    controller.move_to_home()
    
    # Example: run a behavior
    behaviors = controller.get_behavior_files()
    if behaviors:
        print("Running first available behavior:", behaviors[0])
        controller.perform_behavior(behaviors[0])
    
    controller.close()

if __name__ == "__main__":
    main()
