import motor_control
import streamlit as st
from streamlit_extras.stateful_button import button
import os, sys, termios, fcntl

@st.cache_resource
def initQRC():
    return motor_control.MotorControl()


minDuty = 0.000
maxDuty = 100.000
defDuty = minDuty
stepsDuty = 0.005

minFreqHz = 1
maxFreqHz = 100
defFreq = round((minFreqHz + maxFreqHz)/2)
stepsFreq = 1

st.title("Control PWMs")


def parseMotorType(motor_type):
    if motor_type == "BLDC" or motor_type == "LA":
        minDuty = 0.000
        maxDuty = 100.000
        defDuty = minDuty
        stepsDuty = 0.005
        minFreqHz = 1
        maxFreqHz = 100
        defFreq = round((minFreqHz + maxFreqHz)/2)
        stepsFreq = 1
    elif motor_type == "STEP":
        minDuty = 0.000
        maxDuty = 100.000
        defDuty = minDuty
        stepsDuty = 0.005
        minFreqHz = 1
        maxFreqHz = 10000
        defFreq = round((minFreqHz + maxFreqHz)/2)
        stepsFreq = 1
    else:
        minDuty = 0.000
        maxDuty = 1.000
        defDuty = minDuty
        stepsDuty = 0.005
        minFreqHz = 1
        maxFreqHz = 100
        defFreq = round((minFreqHz + maxFreqHz)/2)
        stepsFreq = 1
        print("Not a valid motor type")

    return {
        'minDuty': minDuty,
        'maxDuty': maxDuty,
        'defDuty': defDuty,
        'stepsDuty': stepsDuty,
        'minFreqHz': minFreqHz,
        'maxFreqHz': maxFreqHz,
        'defFreq': defFreq,
        'stepsFreq': stepsFreq
    }

        


class motor:
    def __init__(self, key, motor_type):
        self.key = str(key)

        motor_params = parseMotorType(motor_type)

        if motor_type == "BLDC" or motor_type == "LA":
            self.initBLDC_LA(motor_params)
        
        elif motor_type == "STEP":
            self.initSTEP(motor_params)

        
    def initBLDC_LA(self,motor_params):
        st.header("BLDC_LA Motor " + self.key)
        button_label = "start/stop motor" + self.key
        self.button_key = "b" + self.key
        self.button = button(button_label, key=self.button_key, on_click=self.sendBLDC)
        # hardcode frequency to 50
        self.freq_key = "f" + self.key
        if self.freq_key not in st.session_state:
            st.session_state[self.freq_key] = 50

        
        duty_label = "Duty " + self.key + "%"
        self.duty_key =  "d" + self.key
        self.dutySlider = st.slider(duty_label, minDuty, maxDuty, defDuty, stepsDuty, "%.3f", key=self.duty_key, on_change=self.sendBLDC, disabled = not self.button)

    def initSTEP(self,motor_params):
        st.header("STEPPER Motor " + self.key)

        col1, col2, col3 = st.columns(3)
        with col1: 
            button_label = "start/stop motor" + self.key
            self.button_key = "b" + self.key
            self.button = button(button_label, key=self.button_key, on_click=self.sendBLDC)
        
        with col2:
            self.toggle_key = "t" + self.key
            self.toggle = st.toggle("Lock", key=self.toggle_key,on_change=self.sendStepper)


        with col3:
            self.radio_key = "r" + self.key
            radio = st.radio("Rotation Direction",options=["Clockwise","Counter-Clockwise"],key=self.radio_key,on_change=self.sendStepper, disabled=not self.button)

        freq_label = "Freq " + self.key + "%"
        self.freq_key = "f" + self.key
        self.freqSlider = st.slider(freq_label, minDuty, maxDuty, defDuty, stepsDuty, "%.3f", key=self.freq_key, on_change=self.sendStepper, disabled=not self.button)

        duty_label = "Duty " + self.key + "%"
        self.duty_key =  "d" + self.key
        self.dutySlider = st.slider(duty_label, minDuty, maxDuty, defDuty, stepsDuty, "%.3f", key=self.duty_key, on_change=self.sendStepper, disabled=not self.button)

        



    def sendBLDC(self):
        motor = int(self.key)
        on_off = st.session_state[self.button_key]
        freq = st.session_state[self.freq_key]
        duty = st.session_state[self.duty_key]
        mc.sendBLADCMotorMessage(motor,on_off,duty,freq) #enusre order of freq and duty is correct


    def sendStepper(self):
        motor = int(self.key)
        on_off = st.session_state[self.button_key]
        freq = st.session_state[self.freq_key]
        duty = st.session_state[self.duty_key]
        dir = st.session_state[self.radio_key]
        lock = st.session_state[self.toggle_key]
        dir_val = 0 if dir == "Clockwise" else 1
        lock_val = 0 if lock == False else 1
        mc.sendSTEPMotorMessage(motor,on_off,lock_val,duty,freq,dir_val)
        
        # make mc send message for stepper motors




mc = initQRC()

motor0 = motor(0,"BLDC")
motor1 = motor(1,"BLDC")
motor2 = motor(2,"LA")
motor3 = motor(3,"STEP")
# motor3 = motor(3)
# motor4 = motor(4)
# motor5 = motor(5)
# motor6 = motor(6)
# motor6 = motor(7)


# pwm0.dutySlider = pwm1.dutySlider

# what should this api really look like
# we should create apis for different types of motors...

# it should be types of motors and type of signal we want to send to it.... talk to eric...
# each motor 


# types of motors:
# LA, Stepper, BLDC, 

# or maybe pwm ranges?


# how is LA different from BLDC the UI





