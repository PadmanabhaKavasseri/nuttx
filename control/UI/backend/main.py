# backend/main.py
from fastapi import FastAPI
from fastapi.middleware.cors import CORSMiddleware

#import motor_control python lib

app = FastAPI()

# Allow requests from frontend (running on localhost:3000)
app.add_middleware(
    CORSMiddleware,
    allow_origins=["http://localhost:3000"],
    allow_methods=["*"],
    allow_headers=["*"],
)

@app.get("/ping")
async def ping():
    print("Received ping from frontend")
    return {"message": "pong from FastAPI"}

@app.on_event("startup")
async def startup_event():
    print("🚀 FastAPI app is starting up!")
    #motor control -- init --
    # global mc object
