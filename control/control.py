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

class pwm:
    def __init__(self, key):
        self.key = key
        key=str(key)
        st.header("Motor" + key)
        button_label = "start/stop PWM" + key
        self.button_key = "b" + key
        self.button = button(button_label, key=self.button_key, on_click=self.sendMCB)
        duty_label = "Duty " + key + "%"
        self.duty_key =  "d" + key
        self.dutySlider = st.slider(duty_label, minDuty, maxDuty, defDuty, stepsDuty, "%.3f", key=self.duty_key, on_change=self.sendMCB, disabled = not self.button)
        freq_label = "Frequency " + key
        self.freq_key = "f" + key
        self.freqSlider = st.slider(freq_label, minFreqHz, maxFreqHz, defFreq, stepsFreq, "%f", key=self.freq_key, on_change=self.sendMCB, disabled = not self.button)

    def sendMCB(self):
        motor = self.key
        on_off = st.session_state[self.button_key]
        freq = st.session_state[self.freq_key]
        duty = st.session_state[self.duty_key]

        mc.sendMotorMessage(motor,on_off,duty,freq) #enusre order of freq and duty is correct



mc = initQRC()

pwm0 = pwm(0)
pwm1 = pwm(1)
pwm2 = pwm(2)
pwm3 = pwm(3)
pwm4 = pwm(4)
pwm5 = pwm(5)
pwm6 = pwm(6)
pwm6 = pwm(7)


# pwm0.dutySlider = pwm1.dutySlider
