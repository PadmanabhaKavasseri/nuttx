import streamlit as st
from streamlit_extras.stateful_button import button
import os, sys, termios, fcntl
# import serial
# create home page that allows you to select PWM signals
# for now just one channel will do

minDuty = 5.000
maxDuty = 10.000
defDuty = (minDuty + maxDuty)/2
stepsDuty = 0.005

minFreqHz = 1
maxFreqHz = 100
defFreq = round((minFreqHz + maxFreqHz)/2)
stepsFreq = 1

#open the serial port
# ser = serial.Serial("/dev/qrc")
port = os.open("/dev/qrc", os.O_RDWR|os.O_CREAT)


def makeBmsg(motor, on_off, freq, duty):
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
    dutyBits = format(duty,'014b')

    # Check the lengths of the binary strings
    if len(startBits) > 3 or len(stopBits) > 3 or len(motorBits) > 4 or len(onOffBits) > 1 or len(freqBits) > 7 or len(dutyBits) > 14:
        print("Error: One or more inputs are too large for their respective bit lengths.")
        return None

    binaryMsg = startBits + motorBits + onOffBits + freqBits + dutyBits + stopBits

    return binaryMsg
    
def sendMCB(motor):
    
    binaryMsg = makeBmsg(0,st.session_state.b0,st.session_state.freq0,st.session_state.duty0)
    # print(f"Binary Message: {binaryMsg}")
    st.write("Sent message: ", st.session_state.duty0, st.session_state.freq0)
    # Convert the binary message to bytes
    binaryMsg_bytes = bytes(int(binaryMsg[i : i + 8], 2) for i in range(0, len(binaryMsg), 8))
    binaryMsg_str = ''.join(format(byte, '08b') for byte in binaryMsg_bytes)
    print(f"Binary Message Bytes: {binaryMsg_str}")
    os.write(port,binaryMsg_bytes)

def sendMCBnew(pwm):
    motor = pwm.key
    on_off = st.session_state[pwm.button_key]
    freq = st.session_state[pwm.freq_key]
    duty = st.session_state[pwm.duty_key]

    binaryMsg = makeBmsg(motor, on_off, freq, duty)
    # print(f"Binary Message: {binaryMsg}")
    st.write("Sent message: ", motor, on_off, freq, duty)
    # Convert the binary message to bytes
    binaryMsg_bytes = bytes(int(binaryMsg[i : i + 8], 2) for i in range(0, len(binaryMsg), 8))
    binaryMsg_str = ''.join(format(byte, '08b') for byte in binaryMsg_bytes)
    print(f"Binary Message Bytes: {binaryMsg_str}")
    os.write(port,binaryMsg_bytes)



# b0 = button("start/stop PWM", key="b0", on_click=sendMCB, args=(1,)) #returns true if button is pressed
# duty = st.slider("Duty", minDuty, maxDuty, defDuty, stepsDuty, "%.3f", key="duty0", on_change=sendMCB, args=(1,), disabled= not b0)
# freq = st.slider("Frequency", minFreqHz, maxFreqHz, defFreq, stepsFreq, "%f", key="freq0", on_change=sendMCB, args=(1,), disabled= not b0)

class pwm:
    def __init__(self, key):
        self.key = key
        key=str(key)
        button_label = "start/stop PWM" + key
        self.button_key = "b" + key
        self.button = button(button_label, key=self.button_key, on_click=self.sendMCB)
        duty_label = "Duty " + key + "%"
        self.duty_key =  "d" + key
        self.dutySlider = st.slider(duty_label, minDuty, maxDuty, defDuty, stepsDuty, "%.3f", key=self.duty_key, on_change=self.sendMCB, disabled = not self.button)
        freq_label = "Frequency " + key
        self.freq_key = "f" + key
        self.freqSlider = st.slider(freq_label, minFreqHz, maxFreqHz, defFreq, stepsFreq, "%f", key=self.freq_key, on_change=self.sendMCB, disabled = not self.button)
    
    def sendMCB(pwm):
        motor = pwm.key
        on_off = st.session_state[pwm.button_key]
        freq = st.session_state[pwm.freq_key]
        duty = st.session_state[pwm.duty_key]

        binaryMsg = makeBmsg(motor, on_off, freq, duty)
        # print(f"Binary Message: {binaryMsg}")
        st.write("Sent message: ", motor, on_off, freq, duty)
        # Convert the binary message to bytes
        binaryMsg_bytes = bytes(int(binaryMsg[i : i + 8], 2) for i in range(0, len(binaryMsg), 8))
        binaryMsg_str = ''.join(format(byte, '08b') for byte in binaryMsg_bytes)
        print(f"Binary Message Bytes: {binaryMsg_str}")
        os.write(port,binaryMsg_bytes)

st.title("Control PWMs")

st.header("Motor 1")
pwm1 = pwm(2)

st.header("Motor 2")
pwm2 = pwm(3)


# st.header("Motor 2")

# b0 = button("start/stop PWM", key="b1", on_click=sendMCB, args=(2,)) #returns true if button is pressed
# duty = st.slider("Duty", minDuty, maxDuty, defDuty, stepsDuty, "%.3f", key="duty1", on_change=sendMCB, args=(2,), disabled= not b1)
# freq = st.slider("Frequency", minFreqHz, maxFreqHz, defFreq, stepsFreq, "%f", key="freq1", on_change=sendMCB, args=(2,), disabled= not b1)
