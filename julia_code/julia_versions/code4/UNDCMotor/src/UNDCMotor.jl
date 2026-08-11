# ═══════════════════════════════════════════════════════════════════════════════
#  UNDCMotor.jl – Módulo principal para control e identificación del motor DC.
#  Equivalente al paquete Python (motorsys + controlsys + identsys).
#  LB 2026 – MIT License
# ═══════════════════════════════════════════════════════════════════════════════

module UNDCMotor

# ── Dependencias externas ────────────────────────────────────────────────────
using LibSerialPort
using JSON3
using Interpolations
using DelimitedFiles
using ControlSystems
using LinearAlgebra
using MatrixEquations
using Optim
using Plots
using Statistics
using Printf

# ── Código fuente ────────────────────────────────────────────────────────────
include("motorsys.jl")
include("controlsys.jl")
include("identsys.jl")

# ── API pública ──────────────────────────────────────────────────────────────

# Constantes
export SAMPLING_TIME, BUFFER_SIZE, PRBS_LENGTH

# Tipo principal
export MotorSystemIoT

# Conexión
export connect!, disconnect!

# Conversiones hex (utilidades)
export float2hex, hex2float, long2hex, hex2long
export signal2hex, time2hex, matrix2hex, hexframe_to_array

# Lectura de archivos
export read_csv_file, read_csv_file3

# Comandos de comunicación
export send_command!, receive_frames!

# Modelos de la planta
export transfer_function, speed_from_volts, volts_from_speed

# Funciones de control (controlsys)
export set_reference!, set_pid!, set_controller!
export step_closed, stairs_closed, profile_closed

# Funciones de identificación (identsys)
export step_open, prbs_open
export get_static_model, get_fomodel_step, get_models_prbs

end # module UNDCMotor
