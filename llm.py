import requests
import json

def llm(chat_history):
    url = "http://localhost:11434/api/chat"
    headers = {
        "Content-Type": "application/json"
    }

    data = {
        "model": "trivia",
        "messages": chat_history,
        "stream": False
    }

    try:
        response = requests.post(url, headers=headers, data=json.dumps(data))
        response.raise_for_status()
        json_data = response.json()
        return json_data.get("message", {}).get("content", "").strip()
    except requests.exceptions.RequestException as e:
        print(f"Error connecting to LLM: {e}")
        return "Error connecting to the LLM."
