from fastapi import FastAPI, HTTPException
from fastapi.middleware.cors import CORSMiddleware
from pydantic import BaseModel
from serial_interface import SerialManager

app = FastAPI()
serial = SerialManager()

app.add_middleware(CORSMiddleware, allow_origins=["*"], allow_credentials=True, allow_methods=["*"], allow_headers=["*"])

class Command(BaseModel):
    cmd: str

@app.get("/")
def root():
    return {"status" : "Backend running"}

@app.get("/telemetry")
def get_telem():
    data = serial.get_latest_data()
    return {"telemetry" : data}

@app.get("/command")
def send_command(command : Command):
    try:
        serial.send_command(command.cmd)
        return {"status" : "sent", "command" : command.cmd}
    except Exception as e:
        raise HTTPException(status_code=500, detail=str(e))
    
import atexit
@atexit.register
def cleanup():
    serial.shutdown()