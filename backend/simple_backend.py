from flask import Flask, request, jsonify
from flask_cors import CORS
import json

app = Flask(__name__)
CORS(app)  # This will handle CORS properly

@app.route('/chat/query', methods=['POST'])
def chat_query():
    try:
        data = request.get_json()
        query = data.get('query', '').lower()

        # Simple mock responses
        if 'hello' in query or 'hi' in query:
            answer = "Hello! I'm your Physical AI & Humanoid Robotics textbook assistant. How can I help you with the content?"
        elif 'ros' in query:
            answer = "ROS (Robot Operating System) is a flexible framework for writing robot software. It provides services like hardware abstraction, device drivers, libraries, visualizers, message-passing, and package management for Physical AI systems."
        elif 'humanoid' in query:
            answer = "Humanoid robots are robots with human-like form and capabilities. In Physical AI, they combine perception, decision-making, and action to operate in human environments. The textbook covers kinematics, control systems, and AI integration."
        elif 'physical ai' in query:
            answer = "Physical AI is an interdisciplinary field combining artificial intelligence with physical systems. It focuses on creating intelligent systems that interact with the physical world, integrating machine learning, robotics, and control theory."
        else:
            answer = f"I understand you're asking about '{data.get('query', 'your question')}'. This is a Physical AI & Humanoid Robotics textbook assistant. In the full implementation, this would provide specific textbook content related to your query."

        response = {
            "answer": answer,
            "sources": [{"id": "textbook-chapter", "content": "Relevant textbook content", "source": "Physical AI & Humanoid Robotics Textbook", "score": 0.9}],
            "confidence": 0.85,
            "confidence_level": "high",
            "status": "success"
        }
        return jsonify(response)
    except Exception as e:
        return jsonify({
            "answer": f"Error processing your request: {str(e)}",
            "sources": [],
            "confidence": 0.0,
            "confidence_level": "low",
            "status": "error"
        }), 500

@app.route('/health', methods=['GET'])
def health_check():
    return jsonify({"status": "healthy", "service": "chatbot-backend"})

if __name__ == '__main__':
    app.run(host='0.0.0.0', port=8000, debug=True)