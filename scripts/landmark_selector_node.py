#!/usr/bin/env python

# Python Imports

import os
import sys
from typing import Tuple
from typing import List
from typing import Dict
import numpy as np

import rospy
from rtabmap_ros.srv import *
from rtabmap_ros.msg import Landmark
#from core_test.srv import SelectedLandmarks


from kivy.app import App
from kivymd.uix.button import MDRaisedButton
from kivy.uix.button import Button
from kivymd.uix.floatlayout import MDFloatLayout
from kivy.uix.boxlayout import BoxLayout
from kivy.lang import Builder
from kivy.clock import Clock
from kivy.core.window import Window

Window.clearcolor = (0.4, 0.4, 0.4, 1)
selected_buttons : List[Button] = []
buttons : Dict[int, Tuple[Landmark, Button]] = {}

class LandmarkSelectorApp(App):

    def __init__(self, **kwargs):
        super().__init__(**kwargs)
        
        # self.buttons = {}
        # self.selected_buttons = []
        self.screen = None
        self.bl = None

    def update_landmarks(self):
        try:
            query_service = rospy.ServiceProxy('/rtabmap/landmarks_available', LandmarksQuery)
            resp1 = query_service(LandmarksQueryRequest(False, 20.0))
            
        except rospy.ServiceException as e:
            print("Service call failed: %s"%e)
            for id in buttons.keys():
                #print(id)
                #self.bl.remove_widget(buttons[id][1])
                buttons.pop(int(id))
            
            selected_buttons.clear()
            return
        
        
        self.landmarks = resp1.landmarks

    def create_button(self, landmark):
        
        
        #btn1 = MDRaisedButton(text='Landmark '+str(id), on_press=self.select, md_bg_color=[1,0,0,1])

        #if landmark.landmarkId in self.buttons:
        if landmark.landmarkId in buttons:
            # btn1 = self.buttons[landmark.landmarkId][1]
            btn1 = buttons[landmark.landmarkId][1]
        else:
        
            btn1 = Button(text='Landmark '+str(landmark.landmarkId), background_color=[0,1,1,1], on_press=self.select)
            self.bl.add_widget(btn1)
            btn1.id = landmark.landmarkId

        # self.buttons[landmark.landmarkId] = (landmark, btn1)
        buttons[landmark.landmarkId] = (landmark, btn1)
        
        
       

    def select(self, btn):
        
        loc = int(btn.text.split()[1])
        #if btn in self.selected_buttons:
        if btn in selected_buttons:    
            btn.background_color=[0,1,1,1]
            # self.selected_buttons.remove(btn)
            selected_buttons.remove(btn)
            
        else:
            btn.background_color=[0,1,0.3,1]
            # self.selected_buttons.append(btn)
            selected_buttons.append(btn)
            

        

    def reload_landmarks(self, *args):

        self.update_landmarks()
        landmarkIDs = list(buttons.keys())
        currlandmarkIDs = []
        for landmark in self.landmarks:
            self.create_button(landmark)
            currlandmarkIDs.append(landmark.landmarkId)

        removed_landmarks = [id for id in landmarkIDs if id not in currlandmarkIDs]
        print(removed_landmarks)
            

    def build(self):
        
        # self.screen = BoxLayout(orientation='vertical')
        # reload_btn = MDRaisedButton(text='reload landmarks', on_press=self.reload_landmarks)
        # self.screen.add_widget(reload_btn)
        
        self.bl = BoxLayout(orientation='vertical', spacing=5, padding=10)
        
        Clock.schedule_interval(self.reload_landmarks, 2)
        #self.screen.add_widget(self.bl)
       
        return self.bl

def handle_selected_landmarks_request(req):
    landmarks_res = SelectedLandmarksResponse()
    landmarks_res.landmarks = []
    for button in selected_buttons:
        landmarks_res.landmarks.append(buttons[button.id][0])
    return landmarks_res
    
     
if __name__ == '__main__':

    rospy.init_node('landmark_selector_node', anonymous=False)
    rospy.wait_for_service('/rtabmap/landmarks_available')
    s = rospy.Service('/rtabmap/selected_landmarks', SelectedLandmarks, handle_selected_landmarks_request)

    app = LandmarkSelectorApp()
    app.run()