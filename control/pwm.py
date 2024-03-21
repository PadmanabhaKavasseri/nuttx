import streamlit as st
from streamlit_extras.stateful_button import button
import os, sys, termios, fcntl


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
        

        

    def makeBmsg(self, motor, on_off, freq, duty):
        msg_start = 0
        msg_stop = 7

        # Convert duty from a float to an integer between 5000 and 10000
        duty = int((duty) * 1000)
        print("duty: ", duty)

        startBits = format(msg_start, '03b')
        stopBits = format(msg_stop, '03b')
        motorBits = format(motor, '04b')
        onOffBits = format(on_off, '01b')
        freqBits = format(freq,'07b')
        dutyBits = format(duty,'017b')

        st.write(dutyBits)

        # Check the lengths of the binary strings
        # if len(startBits) > 3 or len(stopBits) > 3 or len(motorBits) > 4 or len(onOffBits) > 1 or len(freqBits) > 7 or len(dutyBits) > 14:
        #     print("Error: One or more inputs are too large for their respective bit lengths.")
        #     return None

        binaryMsg = startBits + motorBits + onOffBits + freqBits + dutyBits + stopBits

        return binaryMsg

    def sendMCB(self):
        motor = self.key
        on_off = st.session_state[self.button_key]
        freq = st.session_state[self.freq_key]
        duty = st.session_state[self.duty_key]

        binaryMsg = self.makeBmsg(motor, on_off, freq, duty)
        # print(f"Binary Message: {binaryMsg}")
        st.write("Sent message: ", motor, on_off, freq, duty)
        # Convert the binary message to bytes
        binaryMsg_bytes = bytes(int(binaryMsg[i : i + 8], 2) for i in range(0, len(binaryMsg), 8))
        binaryMsg_str = ''.join(format(byte, '08b') for byte in binaryMsg_bytes)
        print(f"Binary Message Bytes: {binaryMsg_str}")
        #open the serial port
        port = os.open("/dev/qrc", os.O_RDWR|os.O_CREAT)
        os.write(port,binaryMsg_bytes)


pwm0 = pwm(0)
pwm1 = pwm(1)
pwm2 = pwm(2)
pwm3 = pwm(3)
pwm4 = pwm(4)
pwm5 = pwm(5)
pwm6 = pwm(6)
pwm6 = pwm(7)


# pwm0.dutySlider = pwm1.dutySlider
