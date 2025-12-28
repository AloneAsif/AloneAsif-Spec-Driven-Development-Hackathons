import React, { useState } from "react";
import axios from "axios";
import "./chat.css";

export default function ChatWidget() {
  const [open, setOpen] = useState(false);
  const [messages, setMessages] = useState([]);
  const [input, setInput] = useState("");
  const [loading, setLoading] = useState(false);

  const sendMessage = async () => {
    if (!input.trim() || loading) return;

    const userMessage = { sender: "user", text: input };
    setMessages((prev) => [...prev, userMessage]);
    const currentInput = input;
    setInput("");
    setLoading(true);

    try {
      const res = await axios.post(
        "https://asif007iq-rag-chatbot.hf.space/chat",
        {
          message: currentInput, // Matches FastAPI ChatInput
          selected_text: ""
        },
        {
          headers: { "Content-Type": "application/json" },
          timeout: 25000 
        }
      );

      // Matches the "reply" key returned by your main.py
      const botReply = res.data?.reply || "No answer returned";

      setMessages((prev) => [...prev, { sender: "bot", text: botReply }]);
    } catch (err) {
      const errorMsg = err.response?.data?.reply || err.message || "Server Error";
      setMessages((prev) => [...prev, { sender: "bot", text: `Error: ${errorMsg}` }]);
    } finally {
      setLoading(false);
    }
  };

  return (
    <div className="chat-container">
      <button className="chat-button" onClick={() => setOpen(!open)}>💬 Chat</button>
      {open && (
        <div className="chat-box">
          <div className="chat-header"><strong>AI Assistant</strong></div>
          <div className="chat-body">
            {messages.map((m, i) => <div key={i} className={`bubble ${m.sender}`}>{m.text}</div>)}
            {loading && <div className="bubble bot"><em>Thinking...</em></div>}
          </div>
          <div className="chat-input">
            <input value={input} onChange={(e) => setInput(e.target.value)} onKeyDown={(e) => e.key === "Enter" && sendMessage()} placeholder="Ask me anything..." />
            <button onClick={sendMessage} disabled={loading}>Send</button>
          </div>
        </div>
      )}
    </div>
  );
}