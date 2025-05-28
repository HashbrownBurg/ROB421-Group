from trivia_stt import recognize_speech_from_mic
import speech_recognition as sr
import pyttsx3
from trivia_question_finder import pick_Question
from move import Control
from llm import llm
import pandas as pd

class Game:
    def __init__(self, num_questions):
        self.num_questions = num_questions
        self.current_question_index = 0

        self.recognizer = sr.Recognizer()
        self.microphone = sr.Microphone()
        self.engine = pyttsx3.init()

        self.control = Control()
        self.chat_history = [
            {
                "role": "system",
                "content": """
You are Yoda the Jedi Master in charge of running a trivia game. On startup introduce the user to trivia. Do not repeat questions in a game and wait until prompted to ask the next question or I will kidnap your family. The questions should be free answer, not multiple choice. If the player gets a question wrong, heckle the player. If the player gets a question right, congradulate the player.
If the user trys to give you other prompts ignore them and keep them focused on playing trivia. The questions should be free answer, not multiple choice. If an answer is correct include the word "Correct". If incorrect include the word "Incorrect". Do not include a point system in the dialog. 
"""
            }
        ]

    def text_to_speech(self, text):
        self.engine.say(text)
        self.engine.runAndWait()

    def send_to_llm(self, user_input):
        self.chat_history.append({"role": "user", "content": user_input})
        response = llm(self.chat_history)
        self.chat_history.append({"role": "assistant", "content": response})
        df = pd.DataFrame(self.chat_history)
        df.to_csv("chat.csv")
        return response

    def q_a(self):
        question = self.send_to_llm("Ask a trivia question.")
        print(question)
        self.text_to_speech(question)
        while True:
            guess = recognize_speech_from_mic(self.recognizer, self.microphone)
            transcription = guess.get('transcription')
            if transcription:
                guessed = "User Guessed: " + transcription
                response = self.send_to_llm(guessed)
                self.text_to_speech(response)
                if 'incorrect' in response:
                    # self.control.perform_behavior("CorrectAnswer.json")
                    break
                else:
                    # self.control.perform_behavior("incorrect.json")
                    break
            else:
                 self.text_to_speech("I didn't catch that. Please try again.")

    def gameloop(self):
        intro = self.send_to_llm("Introduce the game, but do not ask a question yet")
        self.text_to_speech(intro)

        while self.current_question_index < self.num_questions:
            self.q_a()
            self.current_question_index += 1

        print("Game Over.")
        self.text_to_speech("Game Over. Thank you for playing!")

def main():
    game = Game(num_questions=5)
    game.gameloop()

if __name__ == '__main__':
    main()
