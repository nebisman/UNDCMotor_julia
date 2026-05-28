# ═══════════════════════════════════════════════════════════════════════════════
#  ejemplo_identificacion.jl – Identificación del sistema UNDCMotor
#
#  Muestra cómo:
#    1. Obtener la curva estática (velocidad vs voltaje)
#    2. Obtener un modelo FOTD desde un escalón
#    3. Obtener un modelo de primer orden desde PRBS
#    4. Obtener la función de transferencia nominal
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

# ── 2. Curva estática ───────────────────────────────────────────────
# NOTA: Este experimento tarda varios minutos.
# Barre todos los voltajes y mide la velocidad estacionaria.
uee, yee = get_static_model(sys; points=30)
println("Curva estática: $(length(uee)) puntos")

# ── 3. Respuesta al escalón en lazo abierto ─────────────────────────
t, u, y = step_open(sys; u0=1.5, u1=3.5, t0=1.0, t1=1.0)
println("Step open: $(length(t)) muestras")

# ── 4. Modelo FOTD desde escalón ────────────────────────────────────
# Estima alpha, tau, L del modelo G(s) = alpha/(tau·s + 1) · exp(-L·s)
alpha, tau, L = get_fomodel_step(sys; yop=400)
println("Modelo FOTD: α=$(round(alpha,digits=3)), τ=$(round(tau,digits=3)), L=$(round(L,digits=3))")

# ── 5. Modelo de primer orden desde PRBS ────────────────────────────
G1 = get_models_prbs(sys; yop=400)
println("Modelo PRBS: $G1")

