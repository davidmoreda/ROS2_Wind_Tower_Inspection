"""Natural language command parsing for the mission controller."""

from __future__ import annotations

import json
import os
import re
import unicodedata
from dataclasses import dataclass


ALLOWED_COMMANDS = {
    'charging_station',
    'maintenance_station',
    'home_inspection',
    'start_inspection',
    'cancel',
    'unknown',
}

COMMAND_LABELS = {
    'charging_station': 'estacion de carga',
    'maintenance_station': 'mantenimiento',
    'home_inspection': 'home inspeccion',
    'start_inspection': 'iniciar inspeccion',
    'cancel': 'cancelar',
    'unknown': 'desconocido',
}


@dataclass(frozen=True)
class CommandIntent:
    command: str
    confidence: float
    reason: str
    source: str


class NaturalLanguageCommandParser:
    """Parse short operator text into one safe mission command."""

    def __init__(self, model: str = 'gemini-2.5-flash'):
        self._model = model

    def parse(self, text: str) -> CommandIntent:
        cleaned = ' '.join((text or '').strip().split())
        if not cleaned:
            return CommandIntent('unknown', 0.0, 'texto vacio', 'local')

        llm_intent = self._parse_with_gemini(cleaned)
        if llm_intent.command != 'unknown' and llm_intent.confidence >= 0.55:
            return llm_intent

        local_intent = self._parse_locally(cleaned)
        if local_intent.command != 'unknown':
            return local_intent

        if llm_intent.reason:
            return llm_intent
        return CommandIntent('unknown', 0.0, 'no se pudo asociar a un comando permitido', 'local')

    def parse_many(self, text: str) -> list[CommandIntent]:
        cleaned = ' '.join((text or '').strip().split())
        if not cleaned:
            return [CommandIntent('unknown', 0.0, 'texto vacio', 'local')]

        segments = [
            segment.strip()
            for segment in re.split(
                r'\b(?:y luego|luego|despues|después|y despues|y después)\b|[;,]',
                cleaned,
                flags=re.IGNORECASE,
            )
            if segment.strip()
        ]
        if len(segments) <= 1:
            return [self.parse(cleaned)]
        return [self.parse(segment) for segment in segments]

    def _parse_with_gemini(self, text: str) -> CommandIntent:
        api_key = os.environ.get('GOOGLE_API_KEY') or os.environ.get('GEMINI_API_KEY')
        if not api_key:
            return CommandIntent('unknown', 0.0, 'GOOGLE_API_KEY/GEMINI_API_KEY no configurada', 'local')

        try:
            from langchain_core.output_parsers import StrOutputParser
            from langchain_core.prompts import ChatPromptTemplate
            from langchain_google_genai import ChatGoogleGenerativeAI
        except ImportError as exc:
            return CommandIntent('unknown', 0.0, f'LangChain/Gemini no instalado: {exc}', 'local')

        prompt = ChatPromptTemplate.from_messages([
            (
                'system',
                'Eres un parser de comandos para un robot ROS 2 de inspeccion. '
                'Devuelve solo JSON valido, sin markdown. '
                'El campo "command" debe ser uno de: '
                'charging_station, maintenance_station, home_inspection, '
                'start_inspection, cancel, unknown. '
                'Usa maintenance_station para frases como "ve a mantenimiento"; '
                'charging_station para "carga", "cargador" o "estacion de carga"; '
                'home_inspection para volver a la base/home/casa de inspeccion; '
                'start_inspection para iniciar o continuar la inspeccion; '
                'cancel para parar, cancelar o e-stop. '
                'Si hay ambiguedad peligrosa, usa unknown. '
                'Formato exacto: '
                '{"command":"maintenance_station","confidence":0.95,"reason":"..."}',
            ),
            ('human', '{operator_text}'),
        ])
        llm = ChatGoogleGenerativeAI(
            model=self._model,
            google_api_key=api_key,
            temperature=0,
        )
        chain = prompt | llm | StrOutputParser()

        try:
            raw = chain.invoke({'operator_text': text})
            data = self._load_json(raw)
        except Exception as exc:  # noqa: BLE001 - keep Flask/ROS node alive on LLM failures
            return CommandIntent('unknown', 0.0, f'Gemini no pudo interpretar: {exc}', 'gemini')

        command = str(data.get('command', 'unknown')).strip()
        if command not in ALLOWED_COMMANDS:
            command = 'unknown'

        try:
            confidence = max(0.0, min(1.0, float(data.get('confidence', 0.0))))
        except (TypeError, ValueError):
            confidence = 0.0

        reason = str(data.get('reason', '')).strip()[:180]
        return CommandIntent(command, confidence, reason, 'gemini')

    @staticmethod
    def _load_json(raw: str) -> dict:
        text = (raw or '').strip()
        if text.startswith('```'):
            text = re.sub(r'^```(?:json)?\s*', '', text)
            text = re.sub(r'\s*```$', '', text)
        return json.loads(text)

    @staticmethod
    def _parse_locally(text: str) -> CommandIntent:
        lowered = _normalize(text)
        rules = [
            (
                'cancel',
                ('cancel', 'cancela', 'para', 'parate', 'detente', 'stop', 'e-stop', 'emergencia'),
                'palabra clave de cancelacion',
            ),
            (
                'maintenance_station',
                ('mantenimiento', 'maintenance', 'sala de control', 'control room', 'taller'),
                'palabra clave de mantenimiento',
            ),
            (
                'charging_station',
                ('carga', 'cargador', 'charging', 'charger', 'bateria', 'battery'),
                'palabra clave de carga',
            ),
            (
                'home_inspection',
                ('home', 'casa', 'base', 'inicio', 'rampa', 'home inspeccion', 'home inspection'),
                'palabra clave de home inspeccion',
            ),
            (
                'start_inspection',
                ('inspecciona', 'inspeccion', 'inspection', 'escanea', 'scan', 'revisa'),
                'palabra clave de inspeccion',
            ),
        ]
        for command, keywords, reason in rules:
            if any(keyword in lowered for keyword in keywords):
                return CommandIntent(command, 0.65, reason, 'local')
        return CommandIntent('unknown', 0.0, 'sin coincidencia local', 'local')


def _normalize(text: str) -> str:
    normalized = unicodedata.normalize('NFKD', text.lower())
    return ''.join(char for char in normalized if not unicodedata.combining(char))
