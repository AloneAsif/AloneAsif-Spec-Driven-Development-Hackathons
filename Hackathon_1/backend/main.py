from fastapi import FastAPI
from pydantic import BaseModel
from engine import ask_bot

app = FastAPI()

class ChatInput(BaseModel):
    message: str
    selected_text: str = None # Frontend sends this if user highlights text

@app.post("/chat")
async def chat_endpoint(data: ChatInput):
    reply = await ask_bot(data.message, data.selected_text)
    return {"reply": reply}

if __name__ == "__main__":
    import uvicorn
    uvicorn.run(app, host="0.0.0.0", port=8000)