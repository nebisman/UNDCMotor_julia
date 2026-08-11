# ═══════════════════════════════════════════════════════════════════════════════
#  ejemplo_controlador.jl – Diseño y carga de controladores al ESP32
#
#  Muestra cómo:
#    1. Diseñar un controlador PID usando ControlSystems.jl
#    2. Cargar un controlador general (espacio de estados) al ESP32
#    3. Verificar la respuesta en lazo cerrado
# ═══════════════════════════════════════════════════════════════════════════════

import Pkg
Pkg.activate(joinpath(@__DIR__, ".."))

using UNDCMotor
using ControlSystems
using Plots

# ── 1. Crear el sistema ─────────────────────────────────────────────
sys = MotorSystemIoT(
    plant_number = "LEO",
    port         = "/dev/ttyUSB0",
    bauds        = 921600,
)

# ── 2. Modelo de la planta (velocidad) ──────────────────────────────
G = transfer_function(sys; output=:speed, min_order=true)
println("Planta: $G")

# ── 3. Diseño de un controlador PI simple ────────────────────────────
s = tf("s")
kp = 0.005; ki = 0.05
C_pi = kp + ki / s                  # PI: C(s) = kp + ki/s

println("Controlador PI: $C_pi")

# Verificar estabilidad en lazo cerrado (simulación)
L = C_pi * G                        # Lazo abierto
T = feedback(L)                     # Lazo cerrado
println("Polos de lazo cerrado: $(poles(T))")

# Diagrama de Bode del lazo abierto
p1 = bodeplot(L; title="Bode del lazo abierto L(s) = C(s)·G(s)")
display(p1)

# Respuesta al escalón simulada
p2 = plot(step(T, 2.0)...; title="Respuesta al escalón simulada",
          xlabel="Tiempo (s)", ylabel="Velocidad (°/s)")
display(p2)

# ── 4. Cargar el controlador PI al ESP32 (1 DOF) ────────────────────
# Para 1 DOF, el controlador opera sobre el error: u = C(s) · (r - y)
set_controller!(sys, C_pi; output=:speed, deadzone=0.0)

# ── 5. Probar con un escalón en lazo cerrado ────────────────────────
t, r, y, u = step_closed(sys; r0=0, r1=400, t0=0.0, t1=3.0)
println("Step completado: $(length(t)) muestras")

# ── 6. Ejemplo de controlador PID con filtro derivativo ──────────────
kp2 = 0.008; ki2 = 0.06; kd2 = 0.0005; Nf = 20.0
C_pid = kp2 + ki2/s + kd2*s / (1 + s/Nf)

println("\nControlador PID con filtro: $C_pid")
set_controller!(sys, C_pid; output=:speed, deadzone=0.0)

t, r, y, u = step_closed(sys; r0=0, r1=400, t0=0.0, t1=3.0)
println("Step con PID filtrado: $(length(t)) muestras")

# ── 7. Ejemplo con controlador 2 DOF ────────────────────────────────
# Controller con 2 entradas: [r, y]
# C₁(s) actúa sobre r, C₂(s) actúa sobre y
# u = C₁(s)·r + C₂(s)·y

beta = 0.7
C1 = beta * kp2 + ki2/s           # rama de referencia (sin derivada)
C2 = -(kp2 + ki2/s + kd2*s/(1 + s/Nf))  # rama de retroalimentación

# Construir el sistema MIMO [1×2]
C_2dof = [C1 C2]
println("\nControlador 2 DOF cargado")
set_controller!(sys, C_2dof; output=:speed, deadzone=0.0)

t, r, y, u = step_closed(sys; r0=0, r1=400, t0=0.0, t1=3.0)
println("Step con 2 DOF: $(length(t)) muestras")
