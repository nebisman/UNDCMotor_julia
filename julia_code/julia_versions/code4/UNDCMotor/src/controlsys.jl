# ═══════════════════════════════════════════════════════════════════════════════
#  controlsys.jl – Funciones de control para el sistema UNDCMotor via serial.
#  Equivalente a controlsys.py.   LB 2026 – MIT License
# ═══════════════════════════════════════════════════════════════════════════════

using Plots
using ControlSystems
using LinearAlgebra
using MatrixEquations
using Printf

# ═══════════════════════════════════════════════════════════════════════════════
#  Utilidades de plotting en tiempo real
# ═══════════════════════════════════════════════════════════════════════════════

"""
    setup_fig(nrows; figsize=(900,500))

Crea una figura con `nrows` subplots verticales usando Plots.jl.
Retorna el objeto plot y un vector de índices de subplot.
En Plots.jl, actualizamos los datos directamente y usamos `display()`.
"""
function setup_fig(nrows::Int; figsize=(900, 500))
    # Se usan layouts de Plots.jl
    plt = plot(layout=(nrows, 1), size=figsize, legend=:bottomright,
               background_color=:white, margin=5Plots.mm)
    display(plt)
    return plt
end

"""Redibuja la figura para actualización en tiempo real."""
function redraw!(plt)
    display(plt)
    return nothing
end

"""Guarda las columnas como CSV (delegado a save_experiment de motorsys)."""
function _save_exp(columns, filename, header)
    save_experiment(columns, filename, header)
end


# ═══════════════════════════════════════════════════════════════════════════════
#  set_reference
# ═══════════════════════════════════════════════════════════════════════════════

"""
    set_reference!(sys, ref_value=50.0)

Fija la referencia del sistema a `ref_value`.
"""
function set_reference!(sys::MotorSystemIoT, ref_value::Real = 50.0)
    connect!(sys)
    send_command!(sys, "set_ref", Dict("reference" => float2hex(ref_value)))
    sleep(0.1)
    disconnect!(sys)
    sleep(1)
    println("Referencia fijada a $(@sprintf("%.2f", ref_value))")
    return nothing
end


# ═══════════════════════════════════════════════════════════════════════════════
#  set_pid
# ═══════════════════════════════════════════════════════════════════════════════

"""
    set_pid!(sys; kp=1.0, ki=0.4, kd=0.0, N=5.0, beta=1.0,
             output=:angle, deadzone=0.125)

Configura los parámetros del controlador PID en el ESP32.
`output` puede ser `:angle` o `:speed`.
"""
function set_pid!(sys::MotorSystemIoT;
                  kp::Real = 1.0, ki::Real = 0.4, kd::Real = 0.0,
                  N::Real = 5.0, beta::Real = 1.0,
                  output::Symbol = :angle, deadzone::Real = 0.125)
    type_map = Dict(:angle => 0, :speed => 1)
    haskey(type_map, output) || error("output debe ser :angle o :speed")

    payload = Dict(
        "kp"          => float2hex(kp),
        "ki"          => float2hex(ki),
        "kd"          => float2hex(kd),
        "N"           => float2hex(N),
        "beta"        => float2hex(beta),
        "typeControl" => long2hex(type_map[output]),
        "deadzone"    => float2hex(deadzone),
    )
    connect!(sys)
    send_command!(sys, "set_pid", payload)
    sleep(0.1)
    disconnect!(sys)
    sleep(1)
    println("Parámetros PID actualizados")
    return nothing
end


# ═══════════════════════════════════════════════════════════════════════════════
#  set_controller  (controlador general en espacio de estados)
# ═══════════════════════════════════════════════════════════════════════════════

"""
    set_controller!(sys, controller; output=:angle, deadzone=0.2)

Carga un controlador general (función de transferencia) al ESP32.
El controlador se convierte a espacio de estados, se discretiza (bilineal/Tustin),
y se calcula la ganancia anti-windup (observador LQR).

`controller` debe ser un TransferFunction de ControlSystems.jl.
- SISO (1 DOF): un solo TF → u = C(s)·(r − y)
- 1×2 MIMO (2 DOF): TF con 2 entradas → u = C₁(s)·r + C₂(s)·y

`output` puede ser `:angle` o `:speed`.
"""
function set_controller!(sys::MotorSystemIoT, controller;
                         output::Symbol = :angle, deadzone::Real = 0.2)
    # Determinar si es 1 DOF o 2 DOF
    ni = size(controller, 2)  # número de entradas

    code_map = Dict(
        (:angle, 1) => 2, (:speed, 1) => 3,
        (:angle, 2) => 4, (:speed, 2) => 5,
    )
    type_control = get(code_map, (output, ni), nothing)
    type_control === nothing && error("output=:angle|:speed, struct 1 ó 2 DOF")

    # ── Construir sistema continuo con 2 entradas [r, y] ────────────
    if ni == 1
        # 1 DOF: u = C(s)·e = C(s)·r − C(s)·y
        G = controller[1, 1]
        sys_1in = ss(G)          # SISO state-space
        A_c  = sys_1in.A
        B_1  = sys_1in.B
        C_c  = sys_1in.C
        D_1  = sys_1in.D
        # [r, y] → u = C(s)·r − C(s)·y
        sys_c = ss(A_c, hcat(B_1, -B_1), C_c, hcat(D_1, -D_1))
    else
        # 2 DOF: u = C₁(s)·r + C₂(s)·y  (ya es un sistema 1×2)
        sys_c = ss(controller)
    end

    # ── Discretización bilineal (Tustin) ──────────────────────────────
    sys_d = c2d(sys_c, SAMPLING_TIME, :tustin)
    Ad = Matrix{Float64}(sys_d.A)
    Bd = Matrix{Float64}(sys_d.B)
    Cd = Matrix{Float64}(sys_d.C)
    Dd = Matrix{Float64}(sys_d.D)
    order = size(Ad, 1)

    # ── Ganancia anti-windup (observador LQR discreto) ────────────────
    # Resolver DARE para la ganancia del observador:
    #   dlqr(A', C', Q, R) → K tal que (A - K·C) es estable
    # Equivale a resolver la DARE dual.
    Q_aw = 10_000.0 * Matrix{Float64}(I, order, order)
    R_aw = ones(1, 1)

    Lg = try
        # DARE: A'XA - X - A'XB(R + B'XB)⁻¹B'XA + Q = 0
        # con A → Ad', B → Cd'
        X, _, _ = ared(Matrix(Ad'), Matrix(Cd'), R_aw, Q_aw)
        K = (R_aw + Cd * X * Cd') \ (Cd * X * Ad')
        Matrix(K')
    catch e
        @warn "No se pudo resolver DARE para anti-windup ($e); usando L = 0"
        zeros(order, 1)
    end

    # Matrices modificadas para el firmware:
    #   x[k+1] = (A − L·C)·x[k] + (B − L·D)·[r, y]' + L·u[k]
    #   u[k]   = C·x[k] + D·[r, y]'
    Ac_fw = Ad - Lg * Cd
    Bc_fw = Bd - Lg * Dd

    payload = Dict(
        "order"       => long2hex(order),
        "A"           => matrix2hex(Ac_fw),
        "B"           => matrix2hex(Bc_fw),
        "C"           => matrix2hex(Cd),
        "D"           => matrix2hex(Dd),
        "L"           => matrix2hex(Lg),
        "typeControl" => long2hex(type_control),
        "deadzone"    => float2hex(deadzone),
    )

    connect!(sys)
    send_command!(sys, "set_gencon", payload)
    sleep(0.1)
    disconnect!(sys)
    sleep(1)
    println("Controlador cargado en UNDCMotor")
    return nothing
end


# ═══════════════════════════════════════════════════════════════════════════════
#  step_closed
# ═══════════════════════════════════════════════════════════════════════════════

"""
    step_closed(sys; r0=0, r1=100, t0=0.0, t1=1.0)

Ejecuta un escalón en lazo cerrado: referencia pasa de `r0` a `r1`.
`t0` = duración en valor bajo, `t1` = duración en valor alto (segundos).
Grafica en tiempo real y retorna (t, r, y, u).
"""
function step_closed(sys::MotorSystemIoT;
                     r0::Real = 0, r1::Real = 100,
                     t0::Real = 0.0, t1::Real = 1.0)
    h  = SAMPLING_TIME
    bs = BUFFER_SIZE
    points_low  = round(Int, t0 / h)
    points_high = round(Int, t1 / h)
    total       = points_low + points_high
    frames      = ceil(Int, total / bs)

    payload = Dict(
        "low_val"     => float2hex(r0),
        "high_val"    => float2hex(r1),
        "points_low"  => long2hex(points_low),
        "points_high" => long2hex(points_high),
    )

    # Vectores acumuladores
    tv, rv, yv, uv = Float64[], Float64[], Float64[], Float64[]

    # Gráfica inicial
    delta = abs(r1 - r0)
    plt = plot(layout=(2, 1), size=(900, 500),
        title=["Step cerrado  r0=$(@sprintf("%.1f",r0)) → r1=$(@sprintf("%.1f",r1))" ""],
        ylabel=["Grados (o °/s)" "Volts"],
        xlabel=["Tiempo (s)" "Tiempo (s)"],
        xlims=[(0, t0 + t1 - h) (0, t0 + t1 - h)],
        ylims=[(min(r0, r1) - 0.6delta, max(r0, r1) + 0.6delta) (-5, 5)],
        background_color_subplot=[:ivory :mintcream],
        legend=[:bottomright :bottomleft],
        grid=true, gridalpha=0.15, margin=5Plots.mm)
    display(plt)

    # Comunicación
    connect!(sys)
    send_command!(sys, "step_closed", payload)

    on_frame = function(msg, frame_no)
        rf = hexframe_to_array(string(msg["r"]))
        uf = hexframe_to_array(string(msg["u"]))
        yf = hexframe_to_array(string(msg["y"]))
        tf = h .* (collect(0:length(rf)-1) .+ (frame_no - 1) * bs)
        append!(rv, rf); append!(yv, yf); append!(uv, uf); append!(tv, tf)

        # Reconstruir gráfica desde cero para evitar trazas duplicadas
        plt = plot(layout=(2, 1), size=(900, 500),
            title=["Step cerrado  r0=$(@sprintf("%.1f",r0)) → r1=$(@sprintf("%.1f",r1))" ""],
            ylabel=["Grados (o °/s)" "Volts"],
            xlabel=["Tiempo (s)" "Tiempo (s)"],
            xlims=[(0, t0 + t1 - h) (0, t0 + t1 - h)],
            ylims=[(min(r0, r1) - 0.6delta, max(r0, r1) + 0.6delta) (-5, 5)],
            background_color_subplot=[:ivory :mintcream],
            legend=[:bottomright :bottomleft],
            grid=true, gridalpha=0.15, margin=5Plots.mm)
        plot!(plt, subplot=1, tv, rv, label="r(t)", color=:teal,
              linewidth=1.25, seriestype=:steppre)
        plot!(plt, subplot=1, tv, yv, label="y(t)", color=:deeppink,
              linewidth=1.0)
        plot!(plt, subplot=2, tv, uv, label="u(t)", color=:royalblue,
              linewidth=1.0)
        redraw!(plt)
    end

    try
        receive_frames!(sys, frames, on_frame)
    catch e
        println("Error: ", e)
    finally
        disconnect!(sys)
    end

    _save_exp([tv, rv, yv, uv], "DCmotor_step_closed_exp.csv", "t,r,y,u")
    return tv, rv, yv, uv
end


# ═══════════════════════════════════════════════════════════════════════════════
#  stairs_closed
# ═══════════════════════════════════════════════════════════════════════════════

"""
    stairs_closed(sys; stairs=(90, 180, 270), duration=1.5)

Ejecuta una señal tipo escaleras en lazo cerrado.
Grafica en tiempo real y retorna (t, r, y, u).
"""
function stairs_closed(sys::MotorSystemIoT;
                       stairs = (90, 180, 270),
                       duration::Real = 1.5)
    h  = SAMPLING_TIME
    bs = BUFFER_SIZE
    stairs_v = collect(Float64, stairs)
    dur_pts  = ceil(Int, duration / h)
    total    = length(stairs_v) * dur_pts - 1
    frames   = ceil(Int, total / bs)
    mn, mx   = minimum(stairs_v), maximum(stairs_v)

    payload = Dict(
        "signal"        => signal2hex(stairs_v),
        "duration"      => long2hex(dur_pts),
        "points_stairs" => long2hex(length(stairs_v)),
        "min_val"       => float2hex(mn),
        "max_val"       => float2hex(mx),
    )

    tv, rv, yv, uv = Float64[], Float64[], Float64[], Float64[]
    span = mx - mn

    plt = plot(layout=(2, 1), size=(900, 500),
        title=["Escaleras  $(length(stairs_v)) niveles, duración $(@sprintf("%.1f",total*h)) s" ""],
        ylabel=["Grados (o °/s)" "Volts"],
        xlabel=["Tiempo (s)" "Tiempo (s)"],
        xlims=[(0, total * h) (0, total * h)],
        ylims=[(min(0, mn - 0.1*abs(span)), mx + 0.1*span) (-5.5, 5.5)],
        background_color_subplot=[:ivory :mintcream],
        legend=:topright, grid=true, gridalpha=0.15, margin=5Plots.mm)
    display(plt)

    connect!(sys)
    send_command!(sys, "stairs_closed", payload)

    on_frame = function(msg, frame_no)
        rf = hexframe_to_array(string(msg["r"]))
        uf = hexframe_to_array(string(msg["u"]))
        yf = hexframe_to_array(string(msg["y"]))
        tf = h .* (collect(0:length(rf)-1) .+ (frame_no - 1) * bs)
        append!(rv, rf); append!(yv, yf); append!(uv, uf); append!(tv, tf)

        plt = plot(layout=(2, 1), size=(900, 500),
            title=["Escaleras  $(length(stairs_v)) niveles, duración $(@sprintf("%.1f",total*h)) s" ""],
            ylabel=["Grados (o °/s)" "Volts"],
            xlabel=["Tiempo (s)" "Tiempo (s)"],
            xlims=[(0, total * h) (0, total * h)],
            ylims=[(min(0, mn - 0.1*abs(span)), mx + 0.1*span) (-5.5, 5.5)],
            background_color_subplot=[:ivory :mintcream],
            legend=:topright, grid=true, gridalpha=0.15, margin=5Plots.mm)
        plot!(plt, subplot=1, tv, rv, label="r(t)", color=:darkgreen,
              linewidth=1.25, seriestype=:steppre)
        plot!(plt, subplot=1, tv, yv, label="y(t)", color=:darkorange,
              linewidth=1.25)
        plot!(plt, subplot=2, tv, uv, label="u(t)", color=:royalblue)
        redraw!(plt)
    end

    try
        receive_frames!(sys, frames, on_frame)
    catch e
        println("Error: ", e)
    finally
        disconnect!(sys)
    end

    _save_exp([tv, rv, yv, uv], "DCmotor_stairs_closed_exp.csv", "t,r,y,u")
    return tv, rv, yv, uv
end


# ═══════════════════════════════════════════════════════════════════════════════
#  profile_closed
# ═══════════════════════════════════════════════════════════════════════════════

"""
    profile_closed(sys; timevalues=(0,1,2,3), refvalues=(0,360,360,0))

Ejecuta un perfil de referencia interpolado en lazo cerrado.
Grafica en tiempo real y retorna (t, r, y, u).
"""
function profile_closed(sys::MotorSystemIoT;
                        timevalues = (0, 1, 2, 3),
                        refvalues  = (0, 360, 360, 0))
    h  = SAMPLING_TIME
    bs = BUFFER_SIZE
    tv_cmd = collect(Float64, timevalues)
    rv_cmd = collect(Float64, refvalues)

    int_tv = [round(Int, p / h) for p in tv_cmd]
    if int_tv[1] != 0
        int_tv  = vcat([0, int_tv[1] - 1], int_tv)
        rv_cmd  = vcat([0.0, 0.0], rv_cmd)
    end

    mn, mx = minimum(rv_cmd), maximum(rv_cmd)
    total  = int_tv[end] + 1
    frames = ceil(Int, total / bs)

    payload = Dict(
        "timevalues" => time2hex(int_tv),
        "refvalues"  => signal2hex(rv_cmd),
        "points"     => long2hex(length(int_tv)),
        "min_val"    => float2hex(mn),
        "max_val"    => float2hex(mx),
    )

    tv, rv, yv, uv = Float64[], Float64[], Float64[], Float64[]
    span = mx - mn

    plt = plot(layout=(2, 1), size=(900, 500),
        title=["Perfil  duración=$(@sprintf("%.1f",tv_cmd[end])) s, $(length(tv_cmd)) puntos" ""],
        ylabel=["Grados (o °/s)" "Volts"],
        xlabel=["Tiempo (s)" "Tiempo (s)"],
        xlims=[(0, (total - 1) * h) (0, (total - 1) * h)],
        ylims=[(mn - 0.1*abs(span), mx + 0.1*span) (-5.5, 5.5)],
        background_color_subplot=[:ivory :mintcream],
        legend=:topright, grid=true, gridalpha=0.15, margin=5Plots.mm)
    display(plt)

    connect!(sys)
    send_command!(sys, "prof_closed", payload)

    on_frame = function(msg, frame_no)
        rf = hexframe_to_array(string(msg["r"]))
        uf = hexframe_to_array(string(msg["u"]))
        yf = hexframe_to_array(string(msg["y"]))
        tf = h .* (collect(0:length(rf)-1) .+ (frame_no - 1) * bs)
        append!(rv, rf); append!(yv, yf); append!(uv, uf); append!(tv, tf)

        plt = plot(layout=(2, 1), size=(900, 500),
            title=["Perfil  duración=$(@sprintf("%.1f",tv_cmd[end])) s, $(length(tv_cmd)) puntos" ""],
            ylabel=["Grados (o °/s)" "Volts"],
            xlabel=["Tiempo (s)" "Tiempo (s)"],
            xlims=[(0, (total - 1) * h) (0, (total - 1) * h)],
            ylims=[(mn - 0.1*abs(span), mx + 0.1*span) (-5.5, 5.5)],
            background_color_subplot=[:ivory :mintcream],
            legend=:topright, grid=true, gridalpha=0.15, margin=5Plots.mm)
        plot!(plt, subplot=1, tv, rv, label="r(t)", color=:darkgreen,
              linewidth=1.25, seriestype=:steppre)
        plot!(plt, subplot=1, tv, yv, label="y(t)", color=:darkorange,
              linewidth=1.25)
        plot!(plt, subplot=2, tv, uv, label="u(t)", color=:royalblue)
        redraw!(plt)
    end

    try
        receive_frames!(sys, frames, on_frame)
    catch e
        println("Error: ", e)
    finally
        disconnect!(sys)
    end

    _save_exp([tv, rv, yv, uv], "DCmotor_profile_closed_exp.csv", "t,r,y,u")
    return tv, rv, yv, uv
end
