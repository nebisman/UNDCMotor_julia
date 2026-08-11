# ═══════════════════════════════════════════════════════════════════════════════
#  setup.jl – Instala todas las dependencias necesarias para UNDCMotor.
#
#  Ejecutar UNA VEZ desde la carpeta UNDCMotor/:
#      julia setup.jl
#  O desde el REPL de Julia:
#      include("/ruta/a/UNDCMotor/setup.jl")
# ═══════════════════════════════════════════════════════════════════════════════

import Pkg

println("═" ^ 60)
println("  Configurando UNDCMotor.jl")
println("═" ^ 60)

# Detectar la carpeta del paquete (donde vive este script)
pkg_dir = @__DIR__

println("\n→ Activando proyecto en: $pkg_dir")
Pkg.activate(pkg_dir)

# Agregar dependencias — Pkg.add resuelve los UUIDs automáticamente
deps = [
    "LibSerialPort",    # Comunicación serial con ESP32
    "JSON3",            # Parsing/serialización JSON
    "Interpolations",   # Interpolación para curva estática
    "DelimitedFiles",   # Lectura/escritura de CSV simples
    "ControlSystems",   # Funciones de transferencia, espacio de estados
    "MatrixEquations",  # Ecuación de Riccati (DARE) para anti-windup
    "Optim",            # Optimización (reemplazo de scipy.optimize)
    "Plots",            # Gráficas en tiempo real
    "Statistics",       # mean, std (stdlib)
    "Printf",          # @sprintf, @printf (stdlib)
    "LinearAlgebra",    # Álgebra lineal (stdlib)
    "CSV",              # Lectura avanzada de CSV
    "DataFrames",       # DataFrames para análisis
]

println("\n→ Instalando $(length(deps)) dependencias...")
for pkg in deps
    print("  • $pkg ... ")
    try
        Pkg.add(pkg)
        println("✓")
    catch e
        println("✗ ($e)")
    end
end

println("\n→ Resolviendo versiones...")
Pkg.resolve()

println("\n→ Precompilando...")
Pkg.precompile()

println("\n" * "═" ^ 60)
println("  ✓ Instalación completada.")
println()
println("  Para usar UNDCMotor:")
println("    import Pkg")
println("    Pkg.activate(\"$pkg_dir\")")
println("    using UNDCMotor")
println()
println("  O bien, para no tener que activar cada vez:")
println("    Pkg.develop(path=\"$pkg_dir\")")
println("    using UNDCMotor")
println("═" ^ 60)
