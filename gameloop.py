from trivia_stt import recognize_speech_from_mic
import speech_recognition as sr
from question import Question
import pyttsx3
from trivia_question_finder import pick_Question
import time
import threading

from move import Control

class Game:
    def __init__(self, num_questions):
        self.num_questions = num_questions
        self.current_question_index = 0
        self.current_question = None

        self.recognizer = sr.Recognizer()
        self.microphone = sr.Microphone()
        self.engine = pyttsx3.init()

        self.control = Control()


    def text_to_speech(self, text):

        def speak(engine, text):
            engine.say(text)
            engine.runAndWait()
        
        threading.Thread(target=speak, args=(self.engine, text)).start()

    def load_question(self):
        return pick_Question('Trivia_Questions.csv')

    def ask_question(self):
        self.text_to_speech(self.current_question.question)
        print(f"Question: {self.current_question.question}")

    def get_guess(self):
        guess = recognize_speech_from_mic(self.recognizer, self.microphone)
        print(f"User said: {guess.get('transcription', '')}")
        return guess

    def check_answer(self, guess):
        transcription = guess.get('transcription')
        if transcription:
            is_correct = self.current_question.correct_answer.lower() == transcription.strip().lower()
            if is_correct:
                self.control.perform_behavior("CorrectAnswer.json")
                response = "The answer is correct!"
            else:
                self.control.perform_behavior("incorrect.json")
                response = "The answer is incorrect."

            self.text_to_speech(response)
            return is_correct
        else:
            self.text_to_speech("I didn't catch that. Please try again.")
            return False

    def gameloop(self):
        self.control.perform_behavior('Wave.json')
        while self.current_question_index < self.num_questions:
            self.current_question = self.load_question()
            self.ask_question()

            while True:
                guess = self.get_guess()
                if self.check_answer(guess):
                    self.current_question_index += 1
                    break

        print("Game Over.")
        self.text_to_speech("Game Over. Thank you for playing!")

def main():
    game = Game(num_questions=5)
    game.gameloop()

if __name__ == '__main__':
    main()
