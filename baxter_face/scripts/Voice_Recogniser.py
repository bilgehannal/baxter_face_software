#!/usr/bin/env python

'''
@Author: Bilgehan NAL
Voice Recogniser is a class to help listening the speech and converting to string
'''

import speech_recognition as sr
import io
import sys

class Voice_Recogniser:

    # These variables are sample language code
    TURKEY = 'tr-TR'
    US = 'en-US'
    UK = 'en-GB'
    FRANCE = 'fr-FR'
    SPAIN = 'es-ES'
    GERMAN = 'de-DE'
    ITALY = 'it-IT'
    RUSSIA = 'ru-RU'

    def __init__(self):
        # Dictionary, all comands are stored in a dictionary
        self.commands = {'---': '---'}

    def __init__(self, txt_path):
        self.commands = {'---': '---'}
        self.updateDictionary(txt_path)

    ''' 
    Dictionary is used for to decide the action
    if the string(speech) includes any key value, it returns command
    txt file should be like below

    hello.be_happy
    wellcome.be_happy

    if we process the string given from listen function.
    The string includes hello or wellcome then, it returns be_happy

    '''
    def updateDictionary(self, path):
        #Read data line by line to a list from the txt file.
        with io.open(path, 'r', encoding='utf-8') as file:
            my_list = file.readlines()
        #Seperating waited speech and commands. These two things are seperated by a character of dot(.)
        for row in my_list:
            command = row.encode('utf-8').split(".")
            if len(command) == 2:
                self.commands[command[0]] = command[1]
                print ("Key: {}, Value: {}".format(command[0], command[1]))

    # listen_language is a voice recognition function, language is given through a parameter.
    def listen_language(self, language):
        string = "-"
        r = sr.Recognizer()
        while string == "-":
            with sr.Microphone() as source:   
                print("Baxter is listening you...")              
                audio = r.listen(source)    
                print("wait...")  
            try:
                string = r.recognize_google(audio, language=language) #Recognize speech
                print("Baxter thinks you said ->  " + string)
            except sr.UnknownValueError:
                string = "-"
            except sr.RequestError as e:
                print("Could not request results from Google Speech Recognition service; {0}".format(e))
        print("Done...")
        return string

    # Default listen function, it recognises US English
    def listen(self):
        string = "-"
        r = sr.Recognizer()
        while string == "-":
            with sr.Microphone() as source:   
                print("Baxter is listening you...")              
                audio = r.listen(source)    
                print("wait...")  
            try:
                string = r.recognize_google(audio, language=US) #Recognize speech
                print("Baxter thinks you said ->  " + string)
            except sr.UnknownValueError:
                string = "-"
            except sr.RequestError as e:
                print("Could not request results from Google Speech Recognition service; {0}".format(e))
        print("Done...")
        return string

    def process(self, string):
        enc = sys.getdefaultencoding()
        result = "Speech could not be processed" #Default message
        string = string.lower()#All cases are converted to lower case
        # Search the commands in dictionary
        for key in self.commands.keys():
            # if the key is substring of string -> key is our commands.
            if key in string:
                result = self.commands[key]
                break
        return result.rstrip().lower().encode('utf_8')