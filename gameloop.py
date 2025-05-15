from espeak_python_easy_interface import text_to_speech
from trivia_stt import recognize_speech_from_mic
import speech_recognition as sr
from question import Question
# import pyttsx3


class Game:
    def __init__(self, num_questions):
        self.num_questions = num_questions
        self.current_question_index = 0
        self.questions = self.load_questions()
        self.current_question = None

        self.recognizer = sr.Recognizer()
        self.microphone = sr.Microphone()

    def load_questions(self):
        return [
            Question(1, 'trivia', 'What is my favorite color?', 'blue'),
            Question(2, 'trivia', 'What planet is known as the Red Planet?', 'mars'),
            Question(3, 'trivia', 'What is the capital of France?', 'paris'),
            Question(4, 'trivia', 'What do bees make?', 'honey'),
            Question(5, 'trivia', 'What gas do plants breathe in?', 'carbon dioxide')
        ]

    def ask_question(self, question):
        # text_to_speech(question.question)
        print(f"Question: {question.question}")

    def get_guess(self):
        guess =  recognize_speech_from_mic(self.recognizer, self.microphone)
        print(guess['transcription'])
        return guess

    def check_answer(self, question, guess):
        if guess['transcription']:
            correct = question.answer == guess['transcription'].strip().lower()
            status = "The Answer is Correct" if correct else "The Answer is Incorrect"
            # text_to_speech(question.question)
            return correct

    def gameloop(self):
        while self.current_question_index < self.num_questions:
            while True:
                self.current_question = self.questions[self.current_question_index]
                self.ask_question(self.current_question)
                guess = self.get_guess()
                correct = self.check_answer(self.current_question, guess)
                if correct:
                    self.current_question_index += 1
                    break

        print("Game Over.")

def main():
    game = Game(num_questions=5)
    game.gameloop()

if __name__ == '__main__':
    main()
