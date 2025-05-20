
import sys
import subprocess
import time

def text_to_speech(text: str, volume: int = 100, pitch: int = 50, speed: int = 175,
                   voice: str = "en", block=True, textFormat: str = "UTF-8"):
    """Python wrapper for espeak. Is based on https://espeak.sourceforge.net/commands.html. Runs
    as a subprocess.
    Inputs:
        text:
            The string which will be spoken
        volume:
            Integer from 0 to 200 representing volume. Is additionally saled by system volume
        pitch:
            Integer from 0 to 99 representing the pitch of the speech
        speed:
            Speed of speech in words per minute (integer).
        voice:
            Voice string used to change between espeak voices. Possible values: en, en+whisper,
            en+croak, en+f1, en+m1
        block:
            bool, if true sleeps the python function until speech is approximately over. If false,
            program continues to run while text is spoken."""

    if (volume < 0) or (volume > 200):
        raise ValueError("Volume must be between 0 and 200")
    
    if (pitch < 0) or (pitch > 99):
        raise ValueError("Pitch must be between 0 and 99")
    
    match textFormat:
        case "UTF-8":
            b = 1
        case "8-bit":
            b = 2
        case "16-bit-unicode" | "16-bit":
            b = 4
        case _:
            raise ValueError("Uknown textFormat string.")


    commandList = ["espeak", "-a", str(volume), "-p", str(pitch), "-s", str(speed), "-v" + str(voice), "-b", str(b), text]
    subprocess.Popen(commandList)

    if block:
        numberOfWords = len(text.split(" "))
        time.sleep(60 / speed * numberOfWords)


if __name__ == "__main__":
    text_to_speech(sys.argv[1])
