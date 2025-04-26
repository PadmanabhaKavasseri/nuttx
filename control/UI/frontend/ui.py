# ui.py

from nicegui import ui
import requests
import sys, os
sys.path.append(os.path.abspath('../../build/.'))

def ping_backend():
    try:
        res = requests.get("http://localhost:3000/ping")
        message = res.json().get("message", "No message")
    except Exception as e:
        message = f"Backend error: {e}"
    ui.notify(message)

from nicegui import ui
from motor_control import MotorControl

mc = MotorControl()

motor_types = {
    'BLDC': {
        'minDuty': 5.000,
        'maxDuty': 10.000,
        'defDuty': 5.000,
        'stepsDuty': 0.005,
        'minFreqHz': 1.0,
        'maxFreqHz': 100,
        'defFreq': 50,
        'stepsFreq': 1,
    },
    'LA': {
        'minDuty': 8.000,
        'maxDuty': 10.000,
        'defDuty': 8.000,
        'stepsDuty': 0.005,
        'minFreqHz': 1.0,
        'maxFreqHz': 100,
        'defFreq': 50,
        'stepsFreq': 1,
    },
    'STEP': {
        'minDuty': 0.000,
        'maxDuty': 100.000,
        'defDuty': 0.000,
        'stepsDuty': 0.005,
        'minFreqHz': 1.0,
        'maxFreqHz': 10000.0,
        'defFreq': 5000,
        'stepsFreq': 1,
    }
}

class MotorUI:
    def __init__(self, motor_id: int, motor_type: str):
        self.id = motor_id
        self.motor_type = motor_type
        self.params = motor_types.get(motor_type, motor_types['STEP'])  # fallback
        self.build_ui()

    def build_ui(self):
        with ui.card().classes('m-4 p-4'):
            ui.label(f"{self.motor_type} Motor {self.id}").classes("text-xl")

            if self.motor_type in ['BLDC', 'LA']:
                self.build_bldc_ui()
            elif self.motor_type == 'STEP':
                self.build_stepper_ui()

    def build_bldc_ui(self):
        ui.label('Duty Cycle (%)')
        self.duty = ui.slider(
            min=self.params['minDuty'],
            max=self.params['maxDuty'],
            step=self.params['stepsDuty'],
            value=self.params['defDuty'],
        ).classes('w-full')

        self.button = ui.toggle(['OFF', 'ON'], value='OFF', on_change=self.send_bldc).props('inline')

        self.duty.on('update:model-value', lambda e: self.send_bldc())

    def build_stepper_ui(self):
        with ui.row():
            with ui.column():
                ui.label('Angle (°)')
                self.angle_input = ui.number(value=0, format='%.0f').props('step="1"')

            with ui.column():
                ui.label('Direction')
                self.direction = ui.radio(['Clockwise', 'Counter-Clockwise'], value='Clockwise')

            with ui.column():
                ui.label('Motor Lock')
                self.lock = ui.toggle(['Free', 'Lock'], value='Free')

        ui.button('Send Step Command', on_click=self.send_stepper)


    def send_bldc(self):
        duty_val = self.duty.value
        on_off = 1 if self.button.value == 'ON' else 0
        freq = self.params['defFreq']
        mc.sendBLADCMotorMessage(self.id, on_off, duty_val, freq)

    def send_stepper(self):
        direction = 0 if self.direction.value == 'Clockwise' else 1
        sleep = True if self.lock.value == 'Lock' else False
        angle = float(self.angle_input.value)
        mc.sendSTEPMotorMessage(self.id, sleep, direction, angle)
        self.angle_input.value = 0

# Initialize UI
ui.label("Motor Control Dashboard").classes("text-2xl m-4")

MotorUI(0, 'LA')
MotorUI(1, 'BLDC')
MotorUI(2, 'BLDC')
MotorUI(3, 'STEP')

# ui.run()



ui.label("🎾 Tennis Ball Machine UI made a change")
ui.button("Ping Backend", on_click=ping_backend)

ui.run(host="0.0.0.0", port=8080)  # Accessible on LAN
