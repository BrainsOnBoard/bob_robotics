#!/usr/bin/env python
import cv2

print("Hello from python!")

edges = []

def my_function(np_array):
    edges = cv2.Canny(np_array, 100, 200)
    cv2.imshow('Python: Ant' 's view', edges)
    cv2.waitKeyEx(1)
