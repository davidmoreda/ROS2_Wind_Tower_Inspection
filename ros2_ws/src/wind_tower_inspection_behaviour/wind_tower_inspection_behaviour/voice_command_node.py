#!/usr/bin/env python3
"""
Voice Command Node — Wind Tower Inspection
==========================================
Captura audio del micrófono, transcribe con faster-whisper (Whisper local)
y publica el texto en el topic /mission_command_text para que el
MissionController lo procese como lenguaje natural.

Dependencias del sistema:
    pip install faster-whisper sounddevice numpy

Para WSL2 se recomienda instalar PulseAudio en el host Windows y configurar:
    export PULSE_SERVER=tcp:$(cat /etc/resolv.conf | grep nameserver | awk '{print $2}')

Parámetros ROS 2:
    model_size     (string, default 'small')   — modelo Whisper: tiny/base/small/medium/large
    device         (string, default 'cpu')     — 'cpu' o 'cuda'
    compute_type   (string, default 'int8')    — cuantización: int8/float16/float32
    language       (string, default 'es')      — código ISO del idioma (es, en, ...)
    silence_thresh (float,  default 0.01)      — umbral RMS para detectar silencio
    silence_sec    (float,  default 1.2)       — segundos de silencio para cortar frase
    min_audio_sec  (float,  default 0.4)       — duración mínima de frase para transcribir
    max_audio_sec  (float,  default 15.0)      — duración máxima de grabación continua
    samplerate     (int,    default 16000)      — frecuencia de muestreo (Hz)
    channels       (int,    default 1)          — canales de audio
    publish_topic  (string, default '/mission_command_text') — topic de salida
"""

from __future__ import annotations

import threading
import queue
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from std_msgs.msg import String, Bool


class VoiceCommandNode(Node):
    """Nodo que graba audio, transcribe con Whisper y publica texto de misión."""

    def __init__(self):
        super().__init__('voice_command_node')

        # ── Parámetros ─────────────────────────────────────────────────────────
        self.declare_parameter('model_size',     'small')
        self.declare_parameter('device',         'cpu')
        self.declare_parameter('compute_type',   'int8')
        self.declare_parameter('language',       'es')
        self.declare_parameter('silence_thresh', 0.01)
        self.declare_parameter('silence_sec',    1.2)
        self.declare_parameter('min_audio_sec',  0.4)
        self.declare_parameter('max_audio_sec',  15.0)
        self.declare_parameter('samplerate',     16000)
        self.declare_parameter('channels',       1)
        self.declare_parameter('publish_topic',  '/mission_command_text')

        self._model_size   = self.get_parameter('model_size').value
        self._device       = self.get_parameter('device').value
        self._compute_type = self.get_parameter('compute_type').value
        self._language     = self.get_parameter('language').value
        self._sil_thresh   = float(self.get_parameter('silence_thresh').value)
        self._sil_sec      = float(self.get_parameter('silence_sec').value)
        self._min_sec      = float(self.get_parameter('min_audio_sec').value)
        self._max_sec      = float(self.get_parameter('max_audio_sec').value)
        self._samplerate   = int(self.get_parameter('samplerate').value)
        self._channels     = int(self.get_parameter('channels').value)
        self._pub_topic    = self.get_parameter('publish_topic').value

        # ── Publishers / Subscribers ───────────────────────────────────────────
        self._text_pub  = self.create_publisher(String, self._pub_topic, 10)
        self._state_pub = self.create_publisher(Bool, '/voice/listening', 10)

        # Suscripción para activar/desactivar escucha desde ROS
        self.create_subscription(Bool, '/voice/enable', self._on_enable, 10)

        # ── Estado interno ──────────────────────────────────────────────────────
        self._listening  = False
        self._model      = None
        self._audio_q: queue.Queue[np.ndarray] = queue.Queue()
        self._lock       = threading.Lock()
        self._listen_thread: threading.Thread | None = None

        # ── Cargar Whisper ──────────────────────────────────────────────────────
        self._load_model()

        # Arrancar escucha continua automáticamente
        self._start_listening()

        self.get_logger().info(
            f'VoiceCommandNode listo — modelo={self._model_size}, '
            f'device={self._device}, idioma={self._language}, '
            f'topic={self._pub_topic}'
        )

    # ── Carga del modelo ───────────────────────────────────────────────────────

    def _load_model(self):
        try:
            from faster_whisper import WhisperModel  # noqa: PLC0415
            self._model = WhisperModel(
                self._model_size,
                device=self._device,
                compute_type=self._compute_type,
            )
            self.get_logger().info(
                f'Modelo faster-whisper "{self._model_size}" cargado '
                f'({self._device}/{self._compute_type})'
            )
        except ImportError:
            self.get_logger().error(
                'faster-whisper no está instalado. '
                'Instálalo con: pip install faster-whisper'
            )
        except Exception as exc:  # noqa: BLE001
            self.get_logger().error(f'Error cargando modelo Whisper: {exc}')

    # ── Control de escucha desde ROS ───────────────────────────────────────────

    def _on_enable(self, msg: Bool):
        if msg.data:
            self._start_listening()
        else:
            self._stop_listening()

    def _start_listening(self):
        with self._lock:
            if self._listening:
                return
            if self._model is None:
                self.get_logger().warn('Modelo no cargado; no se puede escuchar')
                return
            self._listening = True
        self._listen_thread = threading.Thread(
            target=self._listen_loop, daemon=True
        )
        self._listen_thread.start()
        self._publish_state(True)
        self.get_logger().info('Escucha de voz activada')

    def _stop_listening(self):
        with self._lock:
            self._listening = False
        self._publish_state(False)
        self.get_logger().info('Escucha de voz desactivada')

    def _publish_state(self, state: bool):
        self._state_pub.publish(Bool(data=state))

    # ── Bucle principal de captura + transcripción ─────────────────────────────

    def _listen_loop(self):
        """Captura audio del micrófono en bloques; detecta silencios y transcribe."""
        try:
            import sounddevice as sd  # noqa: PLC0415
        except ImportError:
            self.get_logger().error(
                'sounddevice no está instalado. '
                'Instálalo con: pip install sounddevice'
            )
            with self._lock:
                self._listening = False
            self._publish_state(False)
            return

        blocksize = int(self._samplerate * 0.1)   # bloques de 100 ms
        max_samples = int(self._samplerate * self._max_sec)
        min_samples = int(self._samplerate * self._min_sec)
        sil_samples  = int(self._samplerate * self._sil_sec)

        self.get_logger().info(
            f'Capturando audio: samplerate={self._samplerate} Hz, '
            f'blocksize={blocksize} samples'
        )

        try:
            with sd.InputStream(
                samplerate=self._samplerate,
                channels=self._channels,
                dtype='float32',
                blocksize=blocksize,
                callback=self._audio_callback,
            ):
                audio_buf: list[np.ndarray] = []
                silence_buf: list[np.ndarray] = []
                accumulated_silence = 0
                recording = False

                while True:
                    with self._lock:
                        if not self._listening:
                            break
                    try:
                        block = self._audio_q.get(timeout=0.5)
                    except queue.Empty:
                        continue

                    rms = float(np.sqrt(np.mean(block ** 2)))
                    is_voice = rms > self._sil_thresh

                    if is_voice:
                        if not recording:
                            # Prepend los frames de silencio previos (contexto)
                            audio_buf.extend(silence_buf[-3:])
                            recording = True
                        audio_buf.append(block)
                        accumulated_silence = 0
                        silence_buf = []
                    else:
                        if recording:
                            accumulated_silence += len(block)
                            audio_buf.append(block)
                            silence_buf = []

                            # Cortar frase por silencio o duración máxima
                            total = sum(len(b) for b in audio_buf)
                            if (
                                accumulated_silence >= sil_samples
                                or total >= max_samples
                            ):
                                if total >= min_samples:
                                    self._transcribe(audio_buf)
                                audio_buf = []
                                silence_buf = []
                                accumulated_silence = 0
                                recording = False
                        else:
                            # Mantener ventana de silencio reciente para contexto
                            silence_buf.append(block)
                            if len(silence_buf) > 10:
                                silence_buf.pop(0)

        except Exception as exc:  # noqa: BLE001
            self.get_logger().error(f'Error en captura de audio: {exc}')
        finally:
            with self._lock:
                self._listening = False
            self._publish_state(False)

    def _audio_callback(self, indata, frames, time_info, status):  # noqa: ARG002
        if status:
            self.get_logger().warn(f'sounddevice status: {status}', throttle_duration_sec=5.0)
        self._audio_q.put(indata[:, 0].copy() if indata.ndim > 1 else indata.copy())

    # ── Transcripción Whisper ──────────────────────────────────────────────────

    def _transcribe(self, audio_blocks: list[np.ndarray]):
        if self._model is None:
            return

        audio = np.concatenate(audio_blocks).astype(np.float32)
        duration = len(audio) / self._samplerate

        self.get_logger().debug(f'Transcribiendo {duration:.1f}s de audio...')

        try:
            segments, info = self._model.transcribe(
                audio,
                language=self._language,
                beam_size=3,
                vad_filter=True,
                vad_parameters={'min_silence_duration_ms': 300},
            )
            text = ' '.join(seg.text.strip() for seg in segments).strip()
            detected_lang = getattr(info, 'language', self._language)
        except Exception as exc:  # noqa: BLE001
            self.get_logger().error(f'Error en transcripción Whisper: {exc}')
            return

        if not text:
            self.get_logger().debug('Transcripción vacía; ignorando')
            return

        self.get_logger().info(
            f'Transcripción [{detected_lang}]: "{text}"'
        )
        msg = String()
        msg.data = text
        self._text_pub.publish(msg)


# ── Entrypoint ──────────────────────────────────────────────────────────────────

def main(args=None):
    rclpy.init(args=args)
    node = VoiceCommandNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
