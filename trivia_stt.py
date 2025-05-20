
# Speach to text for ROB 421 Trivia game - Team 6

# Note: I saw something saying there is a limited use of how many times you can use this AI 
# per day (something like 50 times) before it stops working. If you start getting an error for no reason, 
# that might be why. - Reilley


import random
import time
import speech_recognition as sr


def recognize_speech_from_mic(recognizer, microphone):
    """Transcribe speech from recorded from `microphone`.

    Returns a dictionary with three keys:
    "success": a boolean indicating whether or not the API request was
               successful
    "error":   `None` if no error occured, otherwise a string containing
               an error message if the API could not be reached or
               speech was unrecognizable
    "transcription": `None` if speech could not be transcribed,
               otherwise a string containing the transcribed text
    """
    # check that recognizer and microphone arguments are appropriate type
    if not isinstance(recognizer, sr.Recognizer):
        raise TypeError("`recognizer` must be `Recognizer` instance")

    if not isinstance(microphone, sr.Microphone):
        raise TypeError("`microphone` must be `Microphone` instance")

    # adjust the recognizer sensitivity to ambient noise and record audio
    # from the microphone
    with microphone as source:
        recognizer.adjust_for_ambient_noise(source)
        audio = recognizer.listen(source)

    # set up the response object
    response = {
        "success": True,
        "error": None,
        "transcription": None
    }

    # try recognizing the speech in the recording
    # if a RequestError or UnknownValueError exception is caught,
    #     update the response object accordingly
    try:
        response["transcription"] = recognizer.recognize_google(audio)
    except sr.RequestError:
        # API was unreachable or unresponsive
        response["success"] = False
        response["error"] = "API unavailable"
    except sr.UnknownValueError:
        # speech was unintelligible
        response["error"] = "Unable to recognize speech"

    return response


if __name__ == "__main__":
    # Temporary questions to show proof of concept
    questions = {"1": "What color is an Orange?",
                 "2": "What color is the sky?"}
    answers = {"1": "Orange",
               "2": "Blue"}
    


    # set the list of words, maxnumber of guesses, and prompt limit
    # WORDS = ["apple", "banana", "grape", "orange", "mango", "lemon"]
    
    NUM_GUESSES = 1  # number of guess attempts
    PROMPT_LIMIT = 5 # number of times to ask to repeat answer if unable to determine what they said

    # create recognizer and mic instances
    recognizer = sr.Recognizer()
    microphone = sr.Microphone()

    # get a random word from the list
    # word = random.choice(WORDS)

    # format the instructions string
    instructions = (
        "Welcome to trivia!\n"
    )

    # show instructions and wait 3 seconds before starting the game
    print(instructions)
    time.sleep(3)

  
    for Q in range(len(questions)):  # for number of questions in list of questions
        for i in range(NUM_GUESSES): # for number of guess attempts
            # get the guess from the user
            # if a transcription is returned, break out of the loop and
            #     continue
            # if no transcription returned and API request failed, break
            #     loop and continue
            # if API request succeeded but no transcription was returned,
            #     re-prompt the user to say their guess again. Do this up
            #     to PROMPT_LIMIT times
            for j in range(PROMPT_LIMIT):
                print(f'{questions[str(Q+1)]}')
                print(f'{answers[str(Q+1)]}')
                print(f'Guess {i+1}. Speak!')
                guess = recognize_speech_from_mic(recognizer, microphone)
                if guess["transcription"]:
                    break
                if not guess["success"]:
                    break
                print("I didn't catch that. What did you say?\n")

            # if there was an error, stop the game
            if guess["error"]:
                print("ERROR: {}".format(guess["error"]))
                break

            # show the user the transcription
            print(f"You said: {guess["transcription"]}")

            # determine if guess is correct and if any attempts remain
            guess_is_correct = guess["transcription"].lower() == answers[str(Q+1)].lower()
            user_has_more_attempts = i < NUM_GUESSES - 1

            # determine if the user has won the game
            # if not, repeat the loop if user has more attempts
            # if no attempts left, the user loses the game
            if guess_is_correct:
                print("Correct!\n")
                break
            elif user_has_more_attempts:
                print("Incorrect. Try again.\n")
            else:
                print(f"Sorry, you lose!\nThe correct answer was '{answers[str(Q+1)]}'.\n")
                break
