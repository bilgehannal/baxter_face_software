#!/usr/bin/env python

"""
@Author: Bilgehan NAL
This file has a class which defines the skin of the baxter's face.
"""

from PIL import Image

class Skin:
    
    indexOfSkin = 5 # choosen element of the array

    def __init__(self, initSkin):
        
        # This array keeps the diffirent colour version of skin
        self.skins = [
            Image.open("data/skin/baxter_skin_0.png"),
            Image.open("data/skin/baxter_skin_1.png"),
            Image.open("data/skin/baxter_skin_2.png"),
            Image.open("data/skin/baxter_skin_3.png"),
            Image.open("data/skin/baxter_skin_4.png"),
            Image.open("data/skin/baxter_skin_5.png")
        ]



        self.indexOfSkin = initSkin
    

    # Encapsulation 

    def setSkin(self, skinNo):
        self.indexOfSkin = skinNo

    def getSkin(self):
        return self.skins[self.indexOfSkin]