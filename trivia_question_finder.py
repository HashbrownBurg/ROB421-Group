
import inquirer
import pandas
import random
from question import Question

def pick_Question(filePath):
    questions = pandas.read_csv(filePath)


    q = questions["Questions"]
    correct = questions["CorrectAnswer"]
    wrong = questions["BadAnswer"]
    opt1 = questions["Answer1"]
    opt2 = questions["Answer2"]
    opt3 = questions["Answer3"]
    opt4 = questions["Answer4"]
    opt5 = questions["Answer5"]
    negResponse = questions["BadResponse"]

    rand = random.randint(0,(len(questions["Questions"])-1))
    question = Question(q[rand], correct[rand], wrong[rand], opt1[rand], opt2[rand], opt3[rand], opt4[rand], opt5[rand], negResponse[rand])
    return question
    
    # options = [opt1.pop(index), opt2.pop(index), opt3.pop(index), opt4.pop(index), opt5.pop(index)]
    # random.shuffle(options)
    # answer = inquirer.list_input(message=Q.pop(index), choices=options)
    
    # badAnswer = wrong.pop(index)
    # goodAnswer = correct.pop(index)
    # rob_response = negResponse.pop(index)
    # if answer == goodAnswer:
    #     print("Wonderful! That is correct!")
    # elif answer == badAnswer:
    #     print(rob_response)
    # else:
    #     print("Unfortunately, that is incorrect.")


if __name__ == "__main__":
    pick_Question(filePath="Trivia_Questions.csv")
