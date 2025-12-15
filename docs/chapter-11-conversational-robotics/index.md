---
title: Chapter 11 - Conversational Robotics
sidebar_position: 11
---

# Conversational Robotics

## Learning Objectives

After completing this chapter, readers will be able to:
1. Implement voice interfaces using Whisper and other speech recognition systems
2. Design effective dialogue management systems for human-robot interaction
3. Create multimodal interaction combining speech, vision, and gesture
4. Apply human-robot interaction design principles
5. Evaluate conversational system performance and user experience

## Voice Interfaces using Whisper

### Speech Recognition Fundamentals

Speech recognition in robotics involves converting acoustic signals to text, which can then be processed by natural language understanding systems. Modern approaches leverage deep learning models trained on large datasets.

### Whisper Architecture and Capabilities

OpenAI's Whisper is a state-of-the-art automatic speech recognition (ASR) system that demonstrates robust performance across multiple languages and acoustic conditions.

#### Whisper Model Variants

- **Tiny**: 39M parameters, suitable for edge deployment
- **Base**: 74M parameters, good balance of accuracy and efficiency
- **Small**: 244M parameters, improved accuracy
- **Medium**: 769M parameters, high accuracy
- **Large**: 1550M parameters, state-of-the-art performance

#### Whisper for Robotics Applications

```python
import whisper
import torch
import pyaudio
import wave
import rospy
from std_msgs.msg import String

class WhisperVoiceInterface:
    def __init__(self, model_size="base"):
        # Load Whisper model
        self.model = whisper.load_model(model_size)

        # Audio parameters
        self.format = pyaudio.paInt16
        self.channels = 1
        self.rate = 16000
        self.chunk = 1024
        self.record_seconds = 5

        # ROS publisher for recognized text
        self.text_pub = rospy.Publisher('recognized_speech', String, queue_size=10)

    def record_audio(self):
        """
        Record audio from microphone
        """
        p = pyaudio.PyAudio()

        stream = p.open(format=self.format,
                        channels=self.channels,
                        rate=self.rate,
                        input=True,
                        frames_per_buffer=self.chunk)

        print("Recording...")
        frames = []

        for i in range(0, int(self.rate / self.chunk * self.record_seconds)):
            data = stream.read(self.chunk)
            frames.append(data)

        print("Finished recording.")

        stream.stop_stream()
        stream.close()
        p.terminate()

        # Save as WAV file for processing
        wf = wave.open("temp_recording.wav", 'wb')
        wf.setnchannels(self.channels)
        wf.setsampwidth(p.get_sample_size(self.format))
        wf.setframerate(self.rate)
        wf.writeframes(b''.join(frames))
        wf.close()

        return "temp_recording.wav"

    def transcribe_audio(self, audio_file):
        """
        Transcribe audio using Whisper
        """
        result = self.model.transcribe(audio_file)
        return result["text"]

    def continuous_listening(self):
        """
        Continuously listen for speech and transcribe
        """
        while not rospy.is_shutdown():
            # Record audio
            audio_file = self.record_audio()

            # Transcribe
            transcription = self.transcribe_audio(audio_file)

            # Publish result
            if transcription.strip():
                msg = String()
                msg.data = transcription
                self.text_pub.publish(msg)
                rospy.loginfo(f"Recognized: {transcription}")

            # Clean up temporary file
            import os
            os.remove(audio_file)

# Example usage in ROS node
def main():
    rospy.init_node('whisper_voice_interface')
    voice_interface = WhisperVoiceInterface(model_size="base")

    try:
        voice_interface.continuous_listening()
    except rospy.ROSInterruptException:
        pass
```

### Optimizing Whisper for Real-time Robotics

#### Latency Considerations
- **Chunked Processing**: Process audio in smaller chunks for reduced latency
- **Model Optimization**: Use smaller models or quantized versions for faster inference
- **Hardware Acceleration**: Leverage GPU acceleration for faster processing

#### Accuracy Improvements
- **Acoustic Adaptation**: Fine-tune models for specific environments
- **Language Model Integration**: Use domain-specific language models
- **Multi-microphone Arrays**: Improve signal-to-noise ratio

### Alternative Speech Recognition Systems

#### Vosk
- **Advantages**: Lightweight, works offline, multiple language support
- **Use Case**: Resource-constrained environments
- **Integration**: Easy Python API for robotics applications

#### Kaldi
- **Advantages**: Highly customizable, research-focused
- **Use Case**: Custom speech recognition systems
- **Integration**: Complex but flexible architecture

#### Google Speech-to-Text
- **Advantages**: High accuracy, cloud-based processing
- **Use Case**: Applications with reliable internet connectivity
- **Integration**: REST API or gRPC interface

## Dialogue Management

### Dialogue System Architecture

A conversational robot's dialogue system typically consists of several components:

#### Speech Recognition
- Converts acoustic signals to text
- Handles acoustic variations and noise

#### Natural Language Understanding (NLU)
- Parses text to extract intent and entities
- Maps to structured representations

#### Dialogue State Tracking
- Maintains context of the conversation
- Tracks user goals and system beliefs

#### Dialogue Policy
- Decides system responses based on state
- Manages conversation flow

#### Natural Language Generation (NLG)
- Converts system responses to natural language
- Ensures natural, contextually appropriate output

### Dialogue Management Approaches

#### Rule-Based Systems
- **Advantages**: Transparent, predictable, controllable
- **Disadvantages**: Limited flexibility, difficult to scale
- **Use Case**: Task-oriented conversations with predictable flows

#### Statistical Systems
- **Advantages**: Learn from data, handle uncertainty
- **Disadvantages**: Require large datasets, less interpretable
- **Use Case**: Conversations with significant variation

#### Neural Systems
- **Advantages**: End-to-end learning, handle complex interactions
- **Disadvantages**: Require massive datasets, limited interpretability
- **Use Case**: Open-domain conversations

### Dialogue State Representation

```python
from dataclasses import dataclass
from typing import Dict, List, Optional
from enum import Enum

class Intent(Enum):
    REQUEST_ACTION = "request_action"
    ASK_INFORMATION = "ask_information"
    CONFIRMATION = "confirmation"
    CANCEL = "cancel"
    GREETING = "greeting"

@dataclass
class Entity:
    type: str
    value: str
    confidence: float

@dataclass
class DialogueState:
    current_intent: Optional[Intent] = None
    entities: List[Entity] = None
    conversation_history: List[Dict[str, str]] = None
    user_goals: List[str] = None
    system_beliefs: Dict[str, float] = None
    turn_count: int = 0

    def __post_init__(self):
        if self.entities is None:
            self.entities = []
        if self.conversation_history is None:
            self.conversation_history = []
        if self.user_goals is None:
            self.user_goals = []
        if self.system_beliefs is None:
            self.system_beliefs = {}

class DialogueManager:
    def __init__(self):
        self.state = DialogueState()
        self.policy_network = self.load_policy_network()

    def update_state(self, user_input: str):
        """
        Update dialogue state based on user input
        """
        # Extract intent and entities
        intent, entities = self.nlu_process(user_input)

        # Update state
        self.state.current_intent = intent
        self.state.entities.extend(entities)

        # Add to conversation history
        self.state.conversation_history.append({
            "speaker": "user",
            "text": user_input,
            "intent": intent.value if intent else None,
            "entities": [(e.type, e.value) for e in entities]
        })

        self.state.turn_count += 1

    def select_action(self) -> str:
        """
        Select appropriate system response based on current state
        """
        # Use policy network to select action
        action = self.policy_network.predict(self.state)

        # Update conversation history
        self.state.conversation_history.append({
            "speaker": "system",
            "text": action,
            "intent": "system_response"
        })

        return action

    def nlu_process(self, text: str):
        """
        Natural Language Understanding processing
        """
        # Simple rule-based NLU for demonstration
        text_lower = text.lower()

        # Intent classification
        if any(word in text_lower for word in ["hello", "hi", "hey"]):
            intent = Intent.GREETING
        elif any(word in text_lower for word in ["please", "could you", "can you"]):
            intent = Intent.REQUEST_ACTION
        elif any(word in text_lower for word in ["what", "how", "where", "when"]):
            intent = Intent.ASK_INFORMATION
        elif any(word in text_lower for word in ["yes", "sure", "ok", "okay"]):
            intent = Intent.CONFIRMATION
        elif any(word in text_lower for word in ["no", "cancel", "stop", "never mind"]):
            intent = Intent.CANCEL
        else:
            intent = None

        # Entity extraction (simplified)
        entities = []
        if "robot" in text_lower:
            entities.append(Entity("agent", "robot", 0.9))
        if any(word in text_lower for word in ["cup", "bottle", "box"]):
            obj = next(word for word in ["cup", "bottle", "box"] if word in text_lower)
            entities.append(Entity("object", obj, 0.8))

        return intent, entities

    def load_policy_network(self):
        """
        Load trained policy network
        """
        # In practice, this would load a trained neural network
        # For demonstration, using a simple rule-based policy
        return SimpleRuleBasedPolicy()

class SimpleRuleBasedPolicy:
    def predict(self, state: DialogueState) -> str:
        """
        Simple rule-based policy for demonstration
        """
        if state.current_intent == Intent.GREETING:
            return "Hello! How can I assist you today?"
        elif state.current_intent == Intent.REQUEST_ACTION:
            if any("object" in ent.type for ent in state.entities):
                obj = next(ent.value for ent in state.entities if ent.type == "object")
                return f"I can help you with the {obj}. What would you like me to do with it?"
            else:
                return "What would you like me to do?"
        elif state.current_intent == Intent.ASK_INFORMATION:
            return "I'm a humanoid robot designed to assist with various tasks. How can I help you?"
        elif state.current_intent == Intent.CONFIRMATION:
            return "Great! I'll proceed with the task."
        elif state.current_intent == Intent.CANCEL:
            return "Okay, I've canceled the current task."
        else:
            return "I'm not sure I understand. Could you please rephrase that?"
```

### Context Management

#### Short-term Context
- Current conversation turn
- Immediate user requests
- System responses and confirmations

#### Long-term Context
- User preferences and history
- Ongoing tasks and goals
- Personalized interactions

#### Context Recovery
- Handling interruptions
- Resuming suspended conversations
- Managing multiple concurrent dialogues

## Multi-modal Interaction (Speech, Vision, Gesture)

### Multi-modal Integration Framework

Multi-modal interaction in robotics combines information from multiple sensory channels to create more natural and effective human-robot communication.

#### Information Fusion Levels

1. **Signal Level Fusion**: Combine raw signals from different modalities
2. **Feature Level Fusion**: Combine extracted features from different modalities
3. **Decision Level Fusion**: Combine decisions made from individual modalities
4. **Semantic Level Fusion**: Combine high-level semantic representations

### Visual-Gestural Communication

```python
import cv2
import numpy as np
from typing import Tuple, List
import rospy
from sensor_msgs.msg import Image
from geometry_msgs.msg import Point
from cv_bridge import CvBridge

class MultiModalInteractionManager:
    def __init__(self):
        self.bridge = CvBridge()
        self.face_cascade = cv2.CascadeClassifier(
            cv2.data.haarcascades + 'haarcascade_frontalface_default.xml'
        )

        # ROS subscribers
        self.image_sub = rospy.Subscriber("/camera/rgb/image_raw", Image, self.image_callback)
        self.speech_sub = rospy.Subscriber("/recognized_speech", String, self.speech_callback)

        # ROS publishers
        self.gesture_pub = rospy.Publisher("/robot/gestures", String, queue_size=10)
        self.attention_pub = rospy.Publisher("/robot/attention_target", Point, queue_size=10)

        # State tracking
        self.current_speaker = None
        self.attention_targets = []

    def image_callback(self, msg):
        """
        Process incoming camera image for visual interaction
        """
        # Convert ROS image to OpenCV
        cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")

        # Detect faces
        gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
        faces = self.face_cascade.detectMultiScale(gray, 1.1, 4)

        # Process each detected face
        for (x, y, w, h) in faces:
            # Calculate face center
            face_center = Point()
            face_center.x = x + w / 2
            face_center.y = y + h / 2
            face_center.z = 0  # Depth would come from depth camera

            # Track face as potential attention target
            self.attention_targets.append(face_center)

            # Draw rectangle around face
            cv2.rectangle(cv_image, (x, y), (x+w, y+h), (255, 0, 0), 2)

        # If we have speech input, direct attention to speaker
        if self.current_speaker:
            self.direct_attention(self.current_speaker)

        # Publish processed image info for other nodes
        # (In practice, you'd likely process this locally)

    def speech_callback(self, msg):
        """
        Process incoming speech for interaction
        """
        speech_text = msg.data.lower()

        # Identify if this is directed at the robot
        if self.is_directed_at_robot(speech_text):
            # Set current speaker (in a real system, you'd use speaker diarization)
            self.current_speaker = self.get_closest_face()

            # Process the speech command
            self.process_speech_command(speech_text)

    def is_directed_at_robot(self, text: str) -> bool:
        """
        Check if speech is directed at the robot
        """
        robot_names = ["robot", "hey robot", "please", "you"]
        return any(name in text for name in robot_names)

    def get_closest_face(self) -> Point:
        """
        Get the closest detected face as attention target
        """
        if self.attention_targets:
            # In a real system, you'd use depth information to determine closest
            # For now, return the first detected face
            return self.attention_targets[0]
        return None

    def direct_attention(self, target: Point):
        """
        Direct robot attention to a specific target
        """
        self.attention_pub.publish(target)

        # Generate appropriate gesture
        gesture_msg = String()
        gesture_msg.data = "gaze_follow"
        self.gesture_pub.publish(gesture_msg)

    def process_speech_command(self, command: str):
        """
        Process speech command and generate appropriate response
        """
        # Update dialogue manager with speech input
        # (Assuming we have a dialogue manager instance)
        # self.dialogue_manager.update_state(command)
        # response = self.dialogue_manager.select_action()

        # For demonstration, simple command processing
        if "wave" in command or "hello" in command:
            gesture_msg = String()
            gesture_msg.data = "wave_hello"
            self.gesture_pub.publish(gesture_msg)
        elif "point" in command and self.current_speaker:
            # Point to the speaker
            gesture_msg = String()
            gesture_msg.data = "point_to_speaker"
            self.gesture_pub.publish(gesture_msg)
```

### Speech-Gesture Coordination

#### Co-speech Gestures
- **Iconic Gestures**: Mimic actions described in speech
- **Deictic Gestures**: Point to objects or locations mentioned in speech
- **Metaphoric Gestures**: Represent abstract concepts spatially

#### Temporal Coordination
- **Simultaneous**: Gesture and speech occur together
- **Sequential**: Gesture precedes or follows speech
- **Complementary**: Gesture provides additional information

### Attention Mechanisms

#### Visual Attention
- **Saliency-based**: Focus on visually prominent objects
- **Goal-directed**: Focus on task-relevant objects
- **Social attention**: Follow human gaze and pointing

#### Audio Attention
- **Sound source localization**: Identify location of sounds
- **Speaker tracking**: Follow specific speaker in multi-person conversations
- **Keyword spotting**: Detect important words or phrases

## Human-Robot Interaction Design

### Interaction Principles

#### Naturalness
- **Conversational Flow**: Follow natural conversation patterns
- **Proactive Interaction**: Initiate interaction when appropriate
- **Context Awareness**: Adapt to situation and environment

#### Intelligibility
- **Clear Communication**: Use understandable language and gestures
- **Feedback Provision**: Provide clear feedback on system state
- **Error Recovery**: Handle and recover from errors gracefully

#### Trust and Safety
- **Predictable Behavior**: Consistent and predictable responses
- **Transparency**: Make system capabilities and limitations clear
- **Safety Assurance**: Prioritize human safety in all interactions

### Design Patterns for HRI

#### Turn-Taking Management
```python
class TurnTakingManager:
    def __init__(self):
        self.human_turn = True
        self.robot_response_queue = []
        self.timeout_duration = 5.0  # seconds
        self.last_speech_time = rospy.Time.now()

    def monitor_conversation(self):
        """
        Monitor conversation flow and manage turn-taking
        """
        current_time = rospy.Time.now()

        # Check if human has finished speaking
        if self.human_turn and self.is_speech_ended():
            self.human_turn = False
            self.initiate_robot_turn()

        # Check for timeout (human not responding)
        elif not self.human_turn and (current_time - self.last_speech_time).to_sec() > self.timeout_duration:
            self.human_turn = True
            self.initiate_human_prompt()

    def is_speech_ended(self) -> bool:
        """
        Determine if human has finished speaking
        """
        # In practice, this would use VAD (Voice Activity Detection)
        # For now, using a simple timeout approach
        current_time = rospy.Time.now()
        return (current_time - self.last_speech_time).to_sec() > 1.0

    def initiate_robot_turn(self):
        """
        Robot takes turn to speak
        """
        if self.robot_response_queue:
            response = self.robot_response_queue.pop(0)
            self.speak(response)

    def initiate_human_prompt(self):
        """
        Robot prompts human to continue conversation
        """
        prompt = "Are you still there? How else can I help you?"
        self.speak(prompt)
        self.human_turn = True
```

#### Social Signal Processing
- **Gaze Behavior**: Appropriate gaze patterns during interaction
- **Proxemics**: Respectful personal space management
- **Gesture Timing**: Appropriate timing and intensity of gestures

#### Adaptive Interaction
- **User Modeling**: Learn and adapt to individual user preferences
- **Context Adaptation**: Adjust behavior based on context
- **Error Adaptation**: Learn from and adapt to errors

### Evaluation Metrics for HRI

#### Task Performance Metrics
- **Task Success Rate**: Percentage of successfully completed tasks
- **Task Completion Time**: Time taken to complete tasks
- **Efficiency**: Ratio of successful actions to total actions

#### Social Interaction Metrics
- **Engagement Duration**: Length of interaction sessions
- **User Satisfaction**: Subjective ratings of interaction quality
- **Naturalness Rating**: How natural the interaction feels

#### Usability Metrics
- **Learnability**: How quickly users learn to interact with the robot
- **Efficiency**: How efficiently users can complete tasks
- **Error Rate**: Frequency of user errors during interaction

### Ethical Considerations in HRI

#### Privacy and Data Protection
- **Data Collection**: Clear policies on what data is collected
- **Data Usage**: Transparent use of collected data
- **Data Storage**: Secure storage and handling of personal data

#### Social Acceptance
- **Cultural Sensitivity**: Respect for cultural differences in interaction
- **Social Norms**: Adherence to social norms and expectations
- **User Autonomy**: Respect for user choices and preferences

#### Safety and Trust
- **Physical Safety**: Ensuring robot actions don't harm humans
- **Psychological Safety**: Avoiding negative psychological impacts
- **Trust Calibration**: Appropriate level of trust in robot capabilities

## Key Claims Requiring Citations

1. Multi-modal interaction combining speech, vision, and gesture creates more natural human-robot communication (Citation needed - see references.md)

2. Whisper provides state-of-the-art automatic speech recognition capabilities for robotics applications (Citation needed - see references.md)

3. Dialogue management systems enable effective task-oriented conversations between humans and robots (Citation needed - see references.md)

4. Proper turn-taking management is essential for natural human-robot interaction (Citation needed - see references.md)

5. Social signal processing improves the quality of human-robot interactions (Citation needed - see references.md)

## Reproducibility Notes

- All examples assume ROS 2 Humble Hawksbill on Ubuntu 22.04 LTS
- Required packages: OpenAI Whisper, PyAudio, OpenCV, ROS audio/sensor packages
- Hardware requirements: RGB camera, microphone array, RTX 3080+ for real-time processing
- Simulation environment: Gazebo with audio/visual sensors

## Summary

This chapter covered conversational robotics, including voice interfaces using Whisper, dialogue management systems, multi-modal interaction combining speech, vision, and gesture, and human-robot interaction design principles. Effective conversational robots require integration of multiple technologies to create natural, intuitive interactions. The next chapter will explore the complete capstone system integrating all components into a functional autonomous humanoid robot.

---

## References

For full citations, see [References](/docs/references.md).