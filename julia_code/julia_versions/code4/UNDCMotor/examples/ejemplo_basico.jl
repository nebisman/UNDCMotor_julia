# ═══════════════════════════════════════════════════════════════════════════════
#  ejemplo_basico.jl – Ejemplo básico de uso de UNDCMotor
#
#  Muestra cómo:
#    1. Crear el objeto del sistema
#    2. Configurar el PID
#    3. Ejecutar un escalón en lazo cerrado
#    4. Ejecutar escaleras en lazo cerrado
# ═══════════════════════════════════════════════════════════════════════════════

# Activar el paquete (ajustar la ruta si es necesario)
import Pkg
Pkg.activate(joinpath(@__DIR__, ".."))

using UNDCMotor

# ── 1. Crear el sistema ─────────────────────────────────────────────
# Ajustar el puerto según el sistema operativo:
#   Linux:   "/dev/ttyUSB0" o "/dev/ttyACM0"
#   macOS:   "/dev/cu.usbserial-XXXX"
#   Windows: "COM3", "COM4", etc.

sys = MotorSystemIoT(
    plant_number = "LEO",
    port         = "/dev/ttyUSB0",
    bauds        = 921600,
)

# ── 2. Configurar el PID para control de posición (ángulo) ──────────
set_pid!(sys;
    kp       = 0.146,
    ki       = 0.732,
    kd       = 0.014,
    N        = 10.0,
    beta     = 0.7,
    output   = :angle,       # :angle para posición, :speed para velocidad
    deadzone = 0.125,
)

# ── 3. Fijar una referencia ─────────────────────────────────────────
set_reference!(sys, 90.0)   # 90 grados

# ── 4. Escalón en lazo cerrado ──────────────────────────────────────
# Referencia salta de 0° a 180° con 0.5 s en bajo y 2 s en alto
t, r, y, u = step_closed(sys; r0=0, r1=180, t0=0.5, t1=2.0)
println("Escalón completado: $(length(t)) muestras adquiridas")

# ── 5. Escaleras en lazo cerrado ────────────────────────────────────
t, r, y, u = stairs_closed(sys;
    stairs   = (0, 90, 180, 270, 180, 90, 0),
    duration = 1.5,
)
println("Escaleras completadas: $(length(t)) muestras")

# ── 6. Perfil arbitrario en lazo cerrado ────────────────────────────
t, r, y, u = profile_closed(sys;
    timevalues = (0, 1, 2, 3, 4, 5),
    refvalues  = (0, 360, 360, 0, -180, 0),
)
println("Perfil completado: $(length(t)) muestras")
