#!/usr/bin/env python3
# Gameplay loop for SAMI trivia bot
# Luke Hashbarger

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class Question:
    def __init__(self, id, category, question, answer):
        self.id = id
        self.category = category
        self.question = question
        self.answer = answer

class Game(Node):
    def __init__(self, num_questions):
        super().__init__('game')
        self.num_questions = num_questions
        self.current_question_index = 0
        self.questions = self.load_questions()
        self.current_question = None
        self.guess = None
        self.listen = False

        self.pub = self.create_publisher(String, 'speaker', 10)
        self.sub = self.create_subscription(String, 'mic', self.get_guess, 10)

        self.timer = self.create_timer(2.0, self.gameloop)

    def load_questions(self):
        return [
            Question(1, 'trivia', 'What is my favorite color?', 'blue'),
            Question(2, 'trivia', 'What planet is known as the Red Planet?', 'mars'),
            Question(3, 'trivia', 'What is the capital of France?', 'paris'),
            Question(4, 'trivia', 'What do bees make?', 'honey'),
            Question(5, 'trivia', 'What gas do plants breathe in?', 'carbon dioxide')
        ]

    def gameloop(self):
        if self.current_question_index >= self.num_questions:
            self.get_logger().info('Game Over.')
            self.timer.cancel()
            return

        if self.current_question is None:
            self.current_question = self.questions[self.current_question_index]
            self.ask_question(self.current_question)
            self.listen = True

        if self.guess:
            self.check_answer(self.current_question, self.guess)
            self.guess = None
            self.current_question = None
            self.current_question_index += 1

    def ask_question(self, question):
        msg = String()
        msg.data = question.question
        self.pub.publish(msg)
        self.get_logger().info(f'Question: {question.question}')

    def get_guess(self, msg):
        if self.listen:
            self.guess = msg.data.strip().lower()
            self.get_logger().info(f'Received guess: {self.guess}')
            self.listen = False

    def check_answer(self, question, guess):
        correct = question.answer.strip().lower() == guess
        msg = String()
        msg.data = "The Answer is Correct" if correct else "The Answer is Incorrect"
        self.pub.publish(msg)
        self.get_logger().info(f'Answer Correct: {correct}')

def main(args=None):
    rclpy.init(args=args)
    game = Game(num_questions=5)
    rclpy.spin(game)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
