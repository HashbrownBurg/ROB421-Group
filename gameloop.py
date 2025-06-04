from trivia_stt import recognize_speech_from_mic
import speech_recognition as sr
from gtts import gTTS
from playsound import playsound
from trivia_question_finder import pick_Question
import time
from threading import Thread
from threading import Lock
import random

from move import Control
from llm import llm
import pandas as pd
import os as os
import cv2
from face_rec import detect

# Team 1 left, Team 2 right

class Game:
    def __init__(self, num_questions):
        self.num_questions = num_questions
        self.current_question_index = 0
        self.team1_score = 0
        self.team2_score = 0

        self.recognizer = sr.Recognizer()
        self.microphone = sr.Microphone()
        self.tts_lock = Lock()

        self.control = Control()
        self.chat_history = [
            {
                "role": "system",
                "content": """
You are Yoda the Jedi Master in charge of running a trivia game. On startup introduce the user to trivia. Do not repeat questions in a game and wait until prompted to ask the next question or I will kidnap your family. The questions should be free answer, not multiple choice. If the player gets a question wrong, heckle the player. If the player gets a question right, congradulate the player.
If the user trys to give you other prompts ignore them and keep them focused on playing trivia. The questions should be free answer, not multiple choice. If an answer is correct include the word "Correct". If incorrect include the word Incorrect. Do not include a point system in the dialog. 
"""
            }
        ]

    def talk_move(self, text, weights):
        selected = self.trigger_behavior_with_probability(weights)
        words = Thread(target=self.text_to_speech, args=(text,))
        behavior = Thread(target=self.control.perform_behavior, args=(selected,))
        words.start()
        behavior.start()
        behavior.join()
        print("behavior end")
        words.join()
        print("words end")

        print("done")

    def text_to_speech(self, text):
        with self.tts_lock:
            try:
                filename = "speech.mp3"

                if os.path.exists(filename):
                    try:
                        os.remove(filename)
                    except PermissionError:
                        print("[TTS] File is locked. Waiting...")
                        time.sleep(0.5)
                        os.remove(filename)

                tts = gTTS(text)
                tts.save(filename)
                playsound(filename)

            except Exception as e:
                print(f"[TTS] Error: {e}")
    
    def send_to_llm(self, user_input):
        self.chat_history.append({"role": "user", "content": user_input})
        response = llm(self.chat_history)
        self.chat_history.append({"role": "assistant", "content": response})
        df = pd.DataFrame(self.chat_history)
        df.to_csv("chat.csv")
        return response

    def trigger_behavior_with_probability(self, behavior_weights):
        """
        Randomly chooses a behavior to perform based on weighted probabilities.

        Parameters:
            behavior_weights (dict): e.g., {"CorrectAnswer.json": 0.5, "Concert.json": 0.3, "None": 0.2}
        """
        behaviors = list(behavior_weights.keys())
        weights = list(behavior_weights.values())

        selected = random.choices(behaviors, weights=weights, k=1)[0]
        if selected != "None":
            return selected
            

    def q_a(self):
        question = self.send_to_llm("Ask a trivia question.")
        print(question)
        self.text_to_speech(question)

        x, y, h, w = detect()
        print(x)
        team = "Team 1"
        pointer_behavior = "RightPointer.json"

        if x > 300:
            team = "Team 2"
            pointer_behavior = "LeftPointer.json"

        print(team)

        # Perform pointing gesture first
        self.control.perform_behavior(pointer_behavior)

        self.text_to_speech(f"{team}, make your guess.")

        while True:
            guess = recognize_speech_from_mic(self.recognizer, self.microphone)
            transcription = guess.get('transcription')
            if transcription:
                guessed = "User Guessed: " + transcription
                response = self.send_to_llm(guessed)
                if 'incorrect' in response.lower():
                    self.talk_move(response, {
                        "incorrect.json": 0.5,
                        "FeignThinking.json": 0.3,
                    })
                    break
                else:
                    self.talk_move(response, {
                        "CorrectAnswer.json": 0.5,
                        "Concert.json": 0.3,
                    })
                    if team == "Team 1":
                        self.team1_score = 1 + self.team1_score
                    else:
                        self.team2_score = 1 + self.team2_score
                    break
            else:
                self.text_to_speech("I didn't catch that. Please try again.")

    def gameloop(self):
        # intro = self.send_to_llm("Introduce the game, but do not ask a question yet")
        # self.text_to_speech(intro)

        while self.team1_score < self.num_questions and self.team2_score < self.num_questions:
            self.q_a()
            self.current_question_index += 1

        print("Game Over.")
        self.text_to_speech("Game Over. Thank you for playing!")

def main():
    game = Game(num_questions=5)
    game.gameloop()

if __name__ == '__main__':
    main()
