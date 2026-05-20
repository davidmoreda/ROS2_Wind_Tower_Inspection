# BUILD_DOCS_PLAN — Plan de modificación de documentación

Fase BUILD. Plan derivado de `docs/audit/BUILD_INPUT.md`. Esta tabla es la guía maestra para esta sesión BUILD.

## Convención de estado

- **IMPLEMENTADO**: existe en código y se lanza/usa.
- **IMPLEMENTADO PARCIALMENTE**: existe pero no integrado o en validación.
- **DEMO / TEST**: existe solo como ejemplo o test.
- **LEGACY PROBABLE**: documentado o disponible pero ya no acorde al pipeline activo.
- **PROPUESTO / NO IMPLEMENTADO**: solo idea o roadmap.
- **NO VERIFICADO**: no se pudo confirmar en esta fase.

---

## Plan por archivo

| Archivo Markdown | Estado actual | Problema | Acción propuesta | Riesgo |
|---|---|---|---|---|
| `README.md` | Desactualizado parcialmente | (1) `cylindrical_map` figura como "EN DESARROLLO" pese a estar implementado; (2) mezcla nodos implementados con propuestos; (3) no menciona la dirección Nav2 cilindro; (4) no apunta a la nueva estructura `docs/` | Reescribir para reflejar estado real + tabla "Estado actual" con marcas IMPLEMENTADO/PROPUESTO + enlaces a `docs/` | ALTO (es la puerta de entrada del repo) |
| `PROJECT_STATE.md` | Vigente con desfase | Fecha 2026-05-09; tabla SLAM/RTAB-Map clasifica `slam.launch.py` solo como "no usado", no como obsoleto | Añadir banner de estado en cabecera redirigiendo a `docs/operation/LAUNCHERS_REFERENCE.md` y `docs/architecture/CURRENT_ARCHITECTURE.md` como fuentes de verdad; NO reescribir contenido | BAJO |
| `PROJECT_PLAN.md` | Roadmap | Mezcla nodos propuestos con implementados; lista mensajes custom inexistentes; muy largo | Añadir banner de estado en cabecera: "Este documento es un ROADMAP histórico/objetivo. Para estado real ver `docs/architecture/CURRENT_ARCHITECTURE.md`" | BAJO |
| `ARCHITECTURE.md` | Vigente | Diagramas precisos; menciona `scan_gate`/`map_accumulator` como prototipos no integrados (correcto) | Añadir banner de estado redirigiendo a `docs/architecture/CURRENT_ARCHITECTURE.md` como fuente canónica futura; conservar contenido | BAJO |
| `LAUNCH_GUIDE.md` | Desactualizado parcialmente | Menciona `map_accumulator` como ejecutable manual; no marca `slam.launch.py` como obsoleto | Añadir banner de estado redirigiendo a `docs/operation/HOW_TO_LAUNCH.md` y `docs/operation/LAUNCHERS_REFERENCE.md`; conservar contenido como referencia operativa detallada | BAJO |
| `CONVERSATION_LESSONS.md` | Útil | Notas históricas vigentes | Conservar tal cual | NO TOCAR |
| `AGENTS.md` (no trackeado) | Vigente | Conciso, sin contradicciones; útil para agentes IA | Pequeña actualización: apuntar a `docs/audit/BUILD_INPUT.md` como referencia obligada y a `docs/` como nueva estructura | BAJO |
| `docs/agent/CONTROL_DEBUG_BRIEF.md` | Vigente | Brief de debug PI | Conservar | NO TOCAR |
| `docs/agent/DEBUG_OUTPUT_POLICY.md` | Vigente | Política de output corto | Conservar | NO TOCAR |
| `docs/agent/OPEN_CODE_WORKFLOW.md` | NO VERIFICADO en profundidad | Pequeña sospecha de obsolescencia | Conservar; no tocar en este BUILD | NO TOCAR |
| `tools/debug/README.md` | Vigente | Describe `capture_inspection_debug.py` | Conservar | NO TOCAR |
| **NUEVO** `docs/architecture/CURRENT_ARCHITECTURE.md` | — | — | Crear: solo lo implementado, con evidencia | MEDIO |
| **NUEVO** `docs/architecture/TARGET_ARCHITECTURE.md` | — | — | Crear: arquitectura objetivo Nav2 cilindro, marcando PROPUESTO | MEDIO |
| **NUEVO** `docs/architecture/NAV2_CYLINDRICAL_NAVIGATION.md` | — | — | Crear: explicación técnica del enfoque cilindro desplegado | MEDIO |
| **NUEVO** `docs/operation/HOW_TO_LAUNCH.md` | — | — | Crear: pasos mínimos para arrancar; condensación útil de `LAUNCH_GUIDE.md` | MEDIO |
| **NUEVO** `docs/operation/LAUNCHERS_REFERENCE.md` | — | — | Crear: tabla canónica de launchers con estado | MEDIO |
| **NUEVO** `docs/operation/NODES_REFERENCE.md` | — | — | Crear: tabla canónica de nodos con estado | MEDIO |
| **NUEVO** `docs/operation/TOPICS_AND_FRAMES.md` | — | — | Crear: lista de topics y frames reales | MEDIO |
| **NUEVO** `docs/development/DEVELOPMENT_GUIDE.md` | — | — | Crear: clonar, ramas, build, source, añadir nodos | MEDIO |
| **NUEVO** `docs/legacy/README.md` | — | — | Crear índice (vacío de momento, sin mover archivos) | BAJO |
| **NUEVO** `docs/audit/BUILD_SUMMARY.md` | — | — | Crear al final con resumen de cambios | BAJO |

---

## Reglas de esta fase BUILD

1. **Conservar todo el código y los launchers** — sin excepción.
2. **Conservar el contenido de docs raíz** (README es la única reescritura grande).
3. **Banners de estado en lugar de borrar**: si un doc raíz queda subsumido por la nueva estructura, se le añade una cabecera de estado que lo indica.
4. **Toda afirmación nueva** debe enlazar a evidencia: ruta de archivo y línea cuando aplique.
5. **Diferenciar siempre** IMPLEMENTADO vs PROPUESTO. Sin metáforas.
