#!/usr/bin/env python3
"""
Speech Recognition Module for Humanoid Robot
Module 4: VLA Systems - Chapter 14

This module provides speech recognition capabilities using both
cloud-based (Whisper API) and local (Vosk) options.
"""

import numpy as np
import sounddevice as sd
from dataclasses import dataclass
from typing import Optional, Callable, Dict, List
from abc import ABC, abstractmethod
import json
import queue
import threading


@dataclass
class AudioConfig:
    """Configuration for audio capture."""
    sample_rate: int = 16000
    channels: int = 1
    dtype: np.dtype = np.float32
    chunk_duration: float = 0.5  # seconds


@dataclass
class TranscriptionResult:
    """Result from speech recognition."""
    text: str
    confidence: float = 1.0
    language: str = "en"
    is_final: bool = True
    segments: List[Dict] = None


class AudioCapture:
    """Capture audio from microphone for speech recognition."""

    def __init__(self, config: AudioConfig = None):
        self.config = config or AudioConfig()
        self.audio_queue = queue.Queue()
        self.is_recording = False
        self.stream = None

    def list_devices(self) -> List[Dict]:
        """List available audio input devices."""
        devices = sd.query_devices()
        return [
            {"index": i, "name": d["name"], "channels": d["max_input_channels"]}
            for i, d in enumerate(devices)
            if d["max_input_channels"] > 0
        ]

    def start_stream(self, callback: Optional[Callable[[np.ndarray], None]] = None):
        """Start continuous audio capture."""
        chunk_size = int(self.config.sample_rate * self.config.chunk_duration)

        def audio_callback(indata, frames, time_info, status):
            if status:
                print(f"Audio status: {status}")
            audio_chunk = indata.copy().flatten()
            if callback:
                callback(audio_chunk)
            else:
                self.audio_queue.put(audio_chunk)

        self.stream = sd.InputStream(
            samplerate=self.config.sample_rate,
            channels=self.config.channels,
            dtype=self.config.dtype,
            blocksize=chunk_size,
            callback=audio_callback
        )
        self.stream.start()
        self.is_recording = True

    def stop_stream(self):
        """Stop audio capture."""
        if self.stream:
            self.stream.stop()
            self.stream.close()
            self.stream = None
        self.is_recording = False

    def record_duration(self, duration: float) -> np.ndarray:
        """Record audio for specified duration."""
        samples = int(self.config.sample_rate * duration)
        recording = sd.rec(
            samples,
            samplerate=self.config.sample_rate,
            channels=self.config.channels,
            dtype=self.config.dtype
        )
        sd.wait()
        return recording.flatten()

    def get_chunk(self, timeout: float = 1.0) -> Optional[np.ndarray]:
        """Get audio chunk from queue."""
        try:
            return self.audio_queue.get(timeout=timeout)
        except queue.Empty:
            return None


class VoiceActivityDetector:
    """Detect speech segments in audio stream."""

    def __init__(
        self,
        energy_threshold: float = 0.01,
        silence_duration: float = 0.5,
        sample_rate: int = 16000
    ):
        self.threshold = energy_threshold
        self.silence_samples = int(silence_duration * sample_rate)
        self.sample_rate = sample_rate

        # State
        self.is_speaking = False
        self.silence_counter = 0
        self.speech_buffer: List[np.ndarray] = []
        self.pre_buffer: List[np.ndarray] = []  # For capturing speech onset
        self.pre_buffer_size = 3  # chunks

    def process_chunk(self, audio_chunk: np.ndarray) -> Dict:
        """Process audio chunk and detect speech boundaries."""
        energy = np.sqrt(np.mean(audio_chunk ** 2))

        result = {
            "is_speech": energy > self.threshold,
            "energy": float(energy),
            "speech_started": False,
            "speech_ended": False,
            "audio": None
        }

        # Maintain pre-buffer
        self.pre_buffer.append(audio_chunk)
        if len(self.pre_buffer) > self.pre_buffer_size:
            self.pre_buffer.pop(0)

        if result["is_speech"]:
            if not self.is_speaking:
                # Speech just started - include pre-buffer
                self.is_speaking = True
                result["speech_started"] = True
                self.speech_buffer = list(self.pre_buffer)

            self.speech_buffer.append(audio_chunk)
            self.silence_counter = 0

        else:
            if self.is_speaking:
                self.silence_counter += len(audio_chunk)
                self.speech_buffer.append(audio_chunk)

                if self.silence_counter > self.silence_samples:
                    # Speech ended
                    self.is_speaking = False
                    result["speech_ended"] = True
                    result["audio"] = np.concatenate(self.speech_buffer)
                    self.speech_buffer = []
                    self.silence_counter = 0

        return result

    def reset(self):
        """Reset detector state."""
        self.is_speaking = False
        self.silence_counter = 0
        self.speech_buffer = []
        self.pre_buffer = []

    def calibrate_threshold(self, audio: np.ndarray, multiplier: float = 2.0):
        """Calibrate threshold based on ambient noise."""
        noise_energy = np.sqrt(np.mean(audio ** 2))
        self.threshold = noise_energy * multiplier
        print(f"Calibrated threshold: {self.threshold:.4f}")


class SpeechRecognizer(ABC):
    """Abstract base class for speech recognizers."""

    @abstractmethod
    def transcribe(self, audio: np.ndarray) -> TranscriptionResult:
        """Transcribe audio to text."""
        pass

    @abstractmethod
    def transcribe_streaming(self, audio_chunk: np.ndarray) -> TranscriptionResult:
        """Process streaming audio chunk."""
        pass


class WhisperTranscriber(SpeechRecognizer):
    """Transcribe audio using OpenAI Whisper (local or API)."""

    def __init__(
        self,
        model_name: str = "base",
        device: str = "cuda",
        use_api: bool = False,
        api_key: str = None
    ):
        self.use_api = use_api
        self.api_key = api_key
        self.model_name = model_name
        self.device = device

        if use_api:
            from openai import OpenAI
            self.client = OpenAI(api_key=api_key)
        else:
            import whisper
            self.model = whisper.load_model(model_name, device=device)

    def transcribe(
        self,
        audio: np.ndarray,
        language: str = "en"
    ) -> TranscriptionResult:
        """Transcribe audio array to text."""
        # Ensure correct format
        if audio.dtype != np.float32:
            audio = audio.astype(np.float32)

        # Normalize
        if np.max(np.abs(audio)) > 1.0:
            audio = audio / np.max(np.abs(audio))

        if self.use_api:
            return self._transcribe_api(audio)
        else:
            return self._transcribe_local(audio, language)

    def _transcribe_local(self, audio: np.ndarray, language: str) -> TranscriptionResult:
        """Transcribe using local Whisper model."""
        result = self.model.transcribe(
            audio,
            language=language,
            fp16=(self.device == "cuda")
        )

        return TranscriptionResult(
            text=result["text"].strip(),
            language=result["language"],
            segments=result.get("segments", []),
            confidence=self._estimate_confidence(result)
        )

    def _transcribe_api(self, audio: np.ndarray) -> TranscriptionResult:
        """Transcribe using Whisper API."""
        import tempfile
        import soundfile as sf

        # Save audio to temporary file
        with tempfile.NamedTemporaryFile(suffix=".wav", delete=False) as f:
            sf.write(f.name, audio, 16000)
            f.flush()

            with open(f.name, "rb") as audio_file:
                response = self.client.audio.transcriptions.create(
                    model="whisper-1",
                    file=audio_file
                )

        return TranscriptionResult(
            text=response.text.strip(),
            confidence=1.0  # API doesn't provide confidence
        )

    def _estimate_confidence(self, result: Dict) -> float:
        """Estimate confidence from Whisper result."""
        if not result.get("segments"):
            return 0.5

        # Average log probability across segments
        avg_logprob = np.mean([
            seg.get("avg_logprob", -1.0)
            for seg in result["segments"]
        ])

        # Convert to confidence (rough approximation)
        confidence = np.exp(avg_logprob)
        return float(np.clip(confidence, 0, 1))

    def transcribe_streaming(self, audio_chunk: np.ndarray) -> TranscriptionResult:
        """Whisper doesn't support true streaming - accumulate and process."""
        # For streaming, you'd want to use faster-whisper or Vosk instead
        return self.transcribe(audio_chunk)


class VoskTranscriber(SpeechRecognizer):
    """Real-time streaming transcription with Vosk."""

    def __init__(self, model_path: str, sample_rate: int = 16000):
        from vosk import Model, KaldiRecognizer

        self.model = Model(model_path)
        self.recognizer = KaldiRecognizer(self.model, sample_rate)
        self.sample_rate = sample_rate

    def transcribe(self, audio: np.ndarray) -> TranscriptionResult:
        """Transcribe complete audio."""
        # Convert to int16 bytes
        audio_bytes = (audio * 32767).astype(np.int16).tobytes()

        # Process all at once
        self.recognizer.AcceptWaveform(audio_bytes)
        result = json.loads(self.recognizer.FinalResult())

        return TranscriptionResult(
            text=result.get("text", ""),
            is_final=True
        )

    def transcribe_streaming(self, audio_chunk: np.ndarray) -> TranscriptionResult:
        """Process streaming audio chunk."""
        # Convert to int16 bytes
        audio_bytes = (audio_chunk * 32767).astype(np.int16).tobytes()

        if self.recognizer.AcceptWaveform(audio_bytes):
            # Final result for this utterance
            result = json.loads(self.recognizer.Result())
            return TranscriptionResult(
                text=result.get("text", ""),
                is_final=True
            )
        else:
            # Partial result
            partial = json.loads(self.recognizer.PartialResult())
            return TranscriptionResult(
                text=partial.get("partial", ""),
                is_final=False
            )

    def reset(self):
        """Reset recognizer for new utterance."""
        from vosk import KaldiRecognizer
        self.recognizer = KaldiRecognizer(self.model, self.sample_rate)


class SpeechRecognitionPipeline:
    """Complete speech recognition pipeline with VAD."""

    def __init__(
        self,
        transcriber: SpeechRecognizer,
        audio_config: AudioConfig = None,
        vad_threshold: float = 0.01
    ):
        self.transcriber = transcriber
        self.audio = AudioCapture(audio_config or AudioConfig())
        self.vad = VoiceActivityDetector(energy_threshold=vad_threshold)
        self.callbacks: List[Callable[[TranscriptionResult], None]] = []
        self._running = False

    def add_callback(self, callback: Callable[[TranscriptionResult], None]):
        """Add callback for transcription results."""
        self.callbacks.append(callback)

    def start(self):
        """Start speech recognition pipeline."""
        self._running = True

        def on_audio(chunk):
            if not self._running:
                return

            vad_result = self.vad.process_chunk(chunk)

            if vad_result["speech_ended"]:
                audio = vad_result["audio"]
                result = self.transcriber.transcribe(audio)

                if result.text:
                    for callback in self.callbacks:
                        callback(result)

        self.audio.start_stream(on_audio)

    def stop(self):
        """Stop speech recognition pipeline."""
        self._running = False
        self.audio.stop_stream()
        self.vad.reset()

    def listen_once(self, timeout: float = 10.0) -> Optional[TranscriptionResult]:
        """Listen for single utterance."""
        result_holder = {"result": None}
        event = threading.Event()

        def on_result(result):
            result_holder["result"] = result
            event.set()

        self.add_callback(on_result)
        self.start()

        event.wait(timeout=timeout)
        self.stop()
        self.callbacks.remove(on_result)

        return result_holder["result"]


# Example usage
if __name__ == "__main__":
    print("Speech Recognition Module Demo")
    print("=" * 40)

    # List audio devices
    audio = AudioCapture()
    print("\nAvailable audio devices:")
    for device in audio.list_devices():
        print(f"  {device['index']}: {device['name']}")

    # Create pipeline with Whisper
    print("\nInitializing speech recognition...")
    try:
        transcriber = WhisperTranscriber(model_name="base", device="cpu")
        pipeline = SpeechRecognitionPipeline(transcriber, vad_threshold=0.02)

        print("\nSpeak something (10 second timeout)...")
        result = pipeline.listen_once(timeout=10.0)

        if result:
            print(f"\nTranscribed: {result.text}")
            print(f"Confidence: {result.confidence:.2f}")
        else:
            print("\nNo speech detected.")

    except ImportError as e:
        print(f"\nNote: {e}")
        print("Install required packages: pip install openai-whisper sounddevice numpy")
