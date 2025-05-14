import inquirer
import pandas
import random


def csvAquirer(filePath="Trivia_Questions.csv"):
    fileInfo = pandas.read_csv(filePath)

    return fileInfo

def pick_Question(questions):

    Q = questions["Questions"]
    correct = questions["CorrectAnswer"]
    wrong = questions["BadAnswer"]
    opt1 = questions["Answer1"]
    opt2 = questions["Answer2"]
    opt3 = questions["Answer3"]
    opt4 = questions["Answer4"]
    opt5 = questions["Answer5"]
    negResponse = questions["BadResponse"]

    numPrompts = list(range(0,len(Q)))
    while len(Q) != 0:
        rand = random.randint(0,(len(questions["Questions"])-1))
        index = numPrompts.pop(rand)
        options = [opt1.pop(index), opt2.pop(index), opt3.pop(index), opt4.pop(index), opt5.pop(index)]
        random.shuffle(options)
        answer = inquirer.list_input(message=Q.pop(index), choices=options)
        
        badAnswer = wrong.pop(index)
        goodAnswer = correct.pop(index)
        rob_response = negResponse.pop(index)

        if answer == goodAnswer:
            print("Wonderful! That is correct!")
        elif answer == badAnswer:
            print(rob_response)
        else:
            print("Unfortunately, that is incorrect.")


if __name__ == "__main__":
    fileInfo = csvAquirer()

    pick_Question(fileInfo)