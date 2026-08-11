# ═══════════════════════════════════════════════════════════════════════════════
#  identsys.jl – Funciones de identificación para el sistema UNDCMotor.
#  Equivalente a identsys.py.   LB 2026 – MIT License
# ═══════════════════════════════════════════════════════════════════════════════

using Plots
using ControlSystems
using LinearAlgebra
using Optim
using Statistics
using Printf

# ═══════════════════════════════════════════════════════════════════════════════
#  step_open  (respuesta escalón en lazo abierto – velocidad)
# ═══════════════════════════════════════════════════════════════════════════════

"""
    step_open(sys; u0=1.5, u1=3.5, t0=1.0, t1=1.0)

Respuesta al escalón en lazo abierto del motor DC (velocidad).
`u0` = voltaje bajo, `u1` = voltaje alto, `t0` y `t1` = duración en segundos.
Grafica en tiempo real y retorna (t, u, y).
"""
function step_open(sys::MotorSystemIoT;
                   u0::Real = 1.5, u1::Real = 3.5,
                   t0::Real = 1.0, t1::Real = 1.0)
    h  = SAMPLING_TIME
    bs = BUFFER_SIZE
    pts_low  = round(Int, t0 / h)
    pts_high = round(Int, t1 / h) + 1
    total    = pts_low + pts_high
    frames   = ceil(Int, total / bs)

    payload = Dict(
        "low_val"     => float2hex(u0),
        "high_val"    => float2hex(u1),
        "points_low"  => long2hex(pts_low),
        "points_high" => long2hex(pts_high),
    )

    tv, uv, yv = Float64[], Float64[], Float64[]

    ulim = sort([u0, u1])
    ylim_v = sort([Float64(speed_from_volts(sys, v)) for v in ulim])
    du = ulim[2] - ulim[1]

    plt = plot(layout=(2, 1), size=(900, 500),
        title=["Step lazo abierto  duración=$(@sprintf("%.2f",total*h)) s" ""],
        ylabel=["Grados/s" "Volts"],
        xlabel=["Tiempo (s)" "Tiempo (s)"],
        xlims=[(0, (total - 1) * h) (0, (total - 1) * h)],
        ylims=[(ylim_v[1] - 50, ylim_v[2] + 50) (ulim[1] - 0.1*du, ulim[2] + 0.1*du)],
        background_color_subplot=[:ivory :mintcream],
        legend=:bottomright, grid=true, gridalpha=0.15, margin=5Plots.mm)
    display(plt)

    connect!(sys)
    send_command!(sys, "step_open", payload)

    on_frame = function(msg, frame_no)
        uf = hexframe_to_array(string(msg["u"]))
        yf = hexframe_to_array(string(msg["y"]))
        tf = h .* (collect(0:length(yf)-1) .+ (frame_no - 1) * bs)
        append!(tv, tf); append!(uv, uf); append!(yv, yf)

        plt = plot(layout=(2, 1), size=(900, 500),
            title=["Step lazo abierto  duración=$(@sprintf("%.2f",total*h)) s" ""],
            ylabel=["Grados/s" "Volts"],
            xlabel=["Tiempo (s)" "Tiempo (s)"],
            xlims=[(0, (total - 1) * h) (0, (total - 1) * h)],
            ylims=[(ylim_v[1] - 50, ylim_v[2] + 50) (ulim[1] - 0.1*du, ulim[2] + 0.1*du)],
            background_color_subplot=[:ivory :mintcream],
            legend=:bottomright, grid=true, gridalpha=0.15, margin=5Plots.mm)
        plot!(plt, subplot=1, tv, yv,
              label="y(t) velocidad", color=:deeppink, linewidth=1.5)
        plot!(plt, subplot=2, tv, uv,
              label="u(t) voltaje", color=:green, linewidth=1.0,
              seriestype=:steppre)
        redraw!(plt)
    end

    try
        receive_frames!(sys, frames, on_frame)
    catch e
        println("Error: ", e)
    finally
        disconnect!(sys)
    end

    _save_exp([tv, uv, yv], "DCmotor_step_open_exp.csv", "t,u,y")
    return tv, uv, yv
end


# ═══════════════════════════════════════════════════════════════════════════════
#  prbs_open  (identificación con señal PRBS en lazo abierto)
# ═══════════════════════════════════════════════════════════════════════════════

"""
    prbs_open(sys; low_val=2.0, high_val=4.0, divider=2)

Identificación en lazo abierto con señal PRBS.
Grafica en tiempo real y retorna (t, u, y).
"""
function prbs_open(sys::MotorSystemIoT;
                   low_val::Real = 2.0, high_val::Real = 4.0,
                   divider::Int = 2)
    h  = SAMPLING_TIME
    bs = BUFFER_SIZE
    total  = PRBS_LENGTH * divider
    frames = ceil(Int, total / bs)

    payload = Dict(
        "low_val"  => float2hex(low_val),
        "high_val" => float2hex(high_val),
        "divider"  => long2hex(divider),
    )

    tv, uv, yv = Float64[], Float64[], Float64[]

    ulim = sort([low_val, high_val])
    ylim_v = sort([Float64(speed_from_volts(sys, v)) for v in ulim])
    du = ulim[2] - ulim[1]

    plt = plot(layout=(2, 1), size=(900, 500),
        title=["PRBS  $total muestras, duración=$(@sprintf("%.2f",total*h)) s" ""],
        ylabel=["Velocidad (°/s)" "Voltaje (V)"],
        xlabel=["" "Tiempo (s)"],
        xlims=[(0, 5*bs*h) (0, 5*bs*h)],
        ylims=[(ylim_v[1] - 25, ylim_v[2] + 25) (ulim[1] - 0.1*du, ulim[2] + 0.1*du)],
        background_color_subplot=[:ivory :mintcream],
        legend=:bottomright, grid=true, gridalpha=0.15, margin=5Plots.mm)
    display(plt)

    connect!(sys)
    send_command!(sys, "prbs_open", payload)

    on_frame = function(msg, frame_no)
        uf = hexframe_to_array(string(msg["u"]))
        yf = hexframe_to_array(string(msg["y"]))
        tf = h .* (collect(0:length(uf)-1) .+ (frame_no - 1) * bs)
        append!(tv, tf); append!(uv, uf); append!(yv, yf)

        # Ventana deslizante después de 6 tramas
        if frame_no > 6
            x_lo = tv[max(1, end - 6*bs + 1)]
            x_hi = tv[end]
            xl = (x_lo, x_hi)
        else
            xl = (0, 5*bs*h)
        end

        plt = plot(layout=(2, 1), size=(900, 500),
            title=["PRBS  $total muestras, duración=$(@sprintf("%.2f",total*h)) s" ""],
            ylabel=["Velocidad (°/s)" "Voltaje (V)"],
            xlabel=["" "Tiempo (s)"],
            xlims=[xl xl],
            ylims=[(ylim_v[1] - 25, ylim_v[2] + 25) (ulim[1] - 0.1*du, ulim[2] + 0.1*du)],
            background_color_subplot=[:ivory :mintcream],
            legend=:bottomright, grid=true, gridalpha=0.15, margin=5Plots.mm)
        plot!(plt, subplot=1, tv, yv, label="y(t)", color=:deeppink,
              seriestype=:steppre)
        plot!(plt, subplot=2, tv, uv, label="PRBS", color=:green,
              seriestype=:steppre)
        redraw!(plt)
    end

    try
        receive_frames!(sys, frames, on_frame)
    catch e
        println("Error: ", e)
    finally
        disconnect!(sys)
    end

    _save_exp([tv, uv, yv], "DCmotor_prbs_open_exp.csv", "t,u,y")
    return tv, uv, yv
end


# ═══════════════════════════════════════════════════════════════════════════════
#  _step_open_static  (step simplificado para curva estática)
# ═══════════════════════════════════════════════════════════════════════════════

"""Step open para curva estática. `sys` ya debe estar conectado."""
function _step_open_static(sys::MotorSystemIoT, u0::Real, u1::Real,
                           t0::Real, t1::Real)
    h  = SAMPLING_TIME
    bs = BUFFER_SIZE
    pts_high = round(Int, t1 / h) + 1
    pts_low  = round(Int, t0 / h)
    total    = pts_low + pts_high
    frames   = ceil(Int, total / bs)

    payload = Dict(
        "low_val"     => float2hex(u0),
        "high_val"    => float2hex(u1),
        "points_low"  => long2hex(pts_low),
        "points_high" => long2hex(pts_high),
    )
    send_command!(sys, "step_open", payload)

    uv, yv = Float64[], Float64[]

    on_frame = function(msg, _fn)
        append!(uv, hexframe_to_array(string(msg["u"])))
        append!(yv, hexframe_to_array(string(msg["y"])))
    end

    receive_frames!(sys, frames, on_frame)
    return uv, yv
end


# ═══════════════════════════════════════════════════════════════════════════════
#  get_static_model  (curva estática del motor)
# ═══════════════════════════════════════════════════════════════════════════════

"""
    get_static_model(sys; points=30)

Obtiene la curva estática velocidad vs voltaje del motor DC.
Retorna (uee, yee) — vectores de voltaje y velocidad estacionaria.
"""
function get_static_model(sys::MotorSystemIoT; points::Int = 30)
    dz_point = 0.3; delta_dz = 0.02; timestep = 3.0
    u_pos = 10.0 .^ range(log10(dz_point - delta_dz), log10(5), length=points)
    u_neg = -reverse(u_pos)
    u_all = vcat(u_neg, u_pos)

    plt = plot(size=(900, 500),
        title="Modelo estático UNDCMotor",
        xlabel="Voltaje (V)", ylabel="Velocidad estacionaria (°/s)",
        xlims=(-5, 5), ylims=(-780, 780),
        background_color=:ivory, grid=true, gridalpha=0.15,
        legend=:topleft, margin=5Plots.mm)
    display(plt)

    uee = Float64[]
    yee = Float64[]

    connect!(sys)
    try
        for ui in u_all
            u_resp, y_resp = try
                _step_open_static(sys, 0, ui, 0, timestep)
            catch
                sleep(4)
                _step_open_static(sys, 0, ui, 0, timestep)
            end

            length(y_resp) < 50 && continue
            yf = Float64(mean(y_resp[end-49:end]))
            uf = Float64(ui)

            # Filtrar puntos en zona muerta
            abs(yf) <= 10 && uf > 0.5 && continue

            push!(uee, uf)
            push!(yee, yf)

            plt = plot(size=(900, 500),
                title="Modelo estático UNDCMotor",
                xlabel="Voltaje (V)", ylabel="Velocidad estacionaria (°/s)",
                xlims=(-5, 5), ylims=(-780, 780),
                background_color=:ivory, grid=true, gridalpha=0.15,
                legend=false, margin=5Plots.mm)
            plot!(plt, uee, yee, color=:green, linewidth=1.5,
                  marker=:circle, markersize=3, label="Curva estática")
            redraw!(plt)
            sleep(0.3)
        end
    finally
        disconnect!(sys)
    end

    exp_data = hcat(uee, yee)
    for path in (PATH_DATA, PATH_DEFAULT)
        open(path * "DCmotor_static_gain_response.csv", "w") do io
            println(io, "u,y")
            for i in axes(exp_data, 1)
                @printf(io, "%.8f,%.8f\n", exp_data[i, 1], exp_data[i, 2])
            end
        end
    end
    println("Modelo estático completado")
    return uee, yee
end


# ═══════════════════════════════════════════════════════════════════════════════
#  get_fomodel_step  (modelo de primer orden desde step)
# ═══════════════════════════════════════════════════════════════════════════════

"""
    get_fomodel_step(sys; yop=400, usefile=false)

Estima un modelo FOTD (First Order plus Time Delay) a partir de un escalón.
Retorna (alpha, tau, L) — ganancia, constante de tiempo, retardo.
"""
function get_fomodel_step(sys::MotorSystemIoT;
                          yop::Real = 400, usefile::Bool = false)
    ymax = Float64(speed_from_volts(sys, 5))
    ymin = Float64(speed_from_volts(sys, -5))
    sigma = 50; timestep = 3

    # Selección del punto de operación
    if -200 < yop < 0
        ua = volts_from_speed(sys, -200)
        ub = volts_from_speed(sys, -100); timestep = 5
    elseif 0 <= yop < 200
        ua = volts_from_speed(sys, 100)
        ub = volts_from_speed(sys, 200); timestep = 5
    elseif ymin <= yop <= ymin + sigma
        ua = volts_from_speed(sys, -5)
        ub = volts_from_speed(sys, yop + sigma)
    elseif ymax - sigma <= yop <= ymax
        ua = volts_from_speed(sys, yop - sigma); ub = 5
    elseif yop >= 200
        ua = volts_from_speed(sys, yop - sigma)
        ub = volts_from_speed(sys, yop + sigma)
    elseif yop <= -200
        ua = volts_from_speed(sys, yop - sigma)
        ub = volts_from_speed(sys, yop + sigma); timestep = 4
    else
        error("Velocidad fuera de rango [$(@sprintf("%.1f",ymin)), $(@sprintf("%.1f",ymax))]")
    end

    if !usefile
        step_open(sys; u0=ua, u1=ub, t0=Float64(timestep), t1=Float64(timestep))
    end

    t, u, y = read_csv_file3(PATH_DATA * "DCmotor_step_open_exp.csv")

    # Detectar el escalón
    ind_step = argmax(diff(u))
    y = y[max(1, ind_step - 50):end]
    u = u[max(1, ind_step - 50):end]
    t = t[max(1, ind_step - 50):end]
    t = t .- t[min(51, length(t))]  # t=0 en el escalón

    ua_d = u[1]
    ub_d = u[end]
    ya = Float64(mean(y[1:min(50, length(y))]))
    yb = Float64(mean(y[max(1, end-49):end]))
    delta_u = ub_d - ua_d
    delta_y = yb - ya

    # Normalizar la respuesta
    y_norm = (y .- ya) ./ delta_y

    # LSQ para estimar tau y L
    yi_pts = [0.1, 0.2, 0.3, 0.4]
    # Interpolación lineal para encontrar los tiempos correspondientes
    ti_pts = Float64[]
    for yp in yi_pts
        idx = findfirst(>=(yp), y_norm)
        if idx === nothing || idx <= 1
            push!(ti_pts, t[1])
        else
            # Interpolación lineal entre idx-1 e idx
            frac = (yp - y_norm[idx-1]) / (y_norm[idx] - y_norm[idx-1])
            push!(ti_pts, t[idx-1] + frac * (t[idx] - t[idx-1]))
        end
    end

    # Sistema lineal: ti = L + tau * (-ln(1 - yi))
    A_lsq = hcat(ones(4), [-log(1 - p) for p in yi_pts])
    # Resolver con restricciones L >= 0, tau >= 0.1
    # Usando mínimos cuadrados simples
    x = A_lsq \ ti_pts
    L_val = max(0.0, x[1])
    tau = max(0.1, x[2])
    alpha = delta_y / delta_u

    # Modelo simulado
    ymodel = [alpha * delta_u * (1 - exp(-max(0, ti - L_val) / tau)) + ya for ti in t]

    # Gráfica
    model_str = @sprintf("G(s) = %.3f/(%.3fs+1)·exp(-%.3fs)", alpha, tau, L_val)

    plt = plot(layout=(2, 1), size=(900, 550),
        title=["Modelo FOTD estimado para UNDCMotor" ""],
        ylabel=["Velocidad (°/s)" "Volts"],
        xlabel=["Tiempo (s)" "Tiempo (s)"],
        xlims=[(-1, timestep) (-1, timestep)],
        background_color_subplot=[:ivory :mintcream],
        legend=:bottomright, grid=true, gridalpha=0.15, margin=5Plots.mm)
    plot!(plt, subplot=1, t, y, label="Datos", color=:teal,
          linewidth=1.5, linestyle=:dot)
    plot!(plt, subplot=1, t, ymodel, label=model_str, color=:deeppink,
          linewidth=1.5)
    plot!(plt, subplot=2, t, u, label="Entrada", color=:green)
    display(plt)

    for path in (PATH_DATA, PATH_DEFAULT)
        open(path * "DCmotor_fomodel_step.csv", "w") do io
            println(io, "alpha,tau,L")
            @printf(io, "%.8f,%.8f,%.8f\n", alpha, tau, L_val)
        end
    end

    return alpha, tau, L_val
end


# ═══════════════════════════════════════════════════════════════════════════════
#  get_models_prbs  (modelo de primer orden desde PRBS)
# ═══════════════════════════════════════════════════════════════════════════════

"""
    get_models_prbs(sys; yop=400, usefile=false)

Estima un modelo de primer orden usando datos de PRBS.
Retorna la función de transferencia G₁(s).
"""
function get_models_prbs(sys::MotorSystemIoT;
                         yop::Real = 400, usefile::Bool = false)
    ymax = Float64(speed_from_volts(sys, 5))
    ymin = Float64(speed_from_volts(sys, -5))
    sigma = 50

    # Punto de operación → voltajes de excitación
    if -200 < yop < 0
        ua = volts_from_speed(sys, -200)
        ub = volts_from_speed(sys, -100)
    elseif 0 <= yop < 200
        ua = volts_from_speed(sys, 200)
        ub = volts_from_speed(sys, 100)
    elseif ymin <= yop <= ymin + sigma
        ua = volts_from_speed(sys, -5)
        ub = volts_from_speed(sys, yop + sigma)
    elseif ymax - sigma <= yop <= ymax
        ua = volts_from_speed(sys, yop - sigma); ub = 5
    elseif 200 <= yop < ymax - sigma
        ua = volts_from_speed(sys, yop + sigma)
        ub = volts_from_speed(sys, yop - sigma)
    elseif ymin + sigma < yop <= -200
        ua = volts_from_speed(sys, yop - sigma)
        ub = volts_from_speed(sys, yop + sigma)
    else
        error("Velocidad fuera de rango [$(@sprintf("%.1f",ymin)), $(@sprintf("%.1f",ymax))]")
    end

    if !usefile
        prbs_open(sys; low_val=ua, high_val=ub, divider=4)
    end

    t_raw, u, y = read_csv_file3(PATH_DATA * "DCmotor_prbs_open_exp.csv")
    ymean_val = Float64(mean(y))
    um = u .- mean(u)
    ym = y .- ymean_val
    delta_y = Float64(speed_from_volts(sys, ub)) - Float64(speed_from_volts(sys, ua))
    delta_u = ub - ua

    # Crear un vector de tiempo uniformemente espaciado (lsim lo requiere)
    N = length(t_raw)
    dt = (t_raw[end] - t_raw[1]) / (N - 1)
    t = collect(range(t_raw[1], step=dt, length=N))

    # Formatear entrada como matriz (1 × N) para lsim
    u_matrix = reshape(um, 1, :)

    # ── Modelo de primer orden: G(s) = alpha / (tau·s + 1) ──────────
    alpha0 = delta_y / delta_u

    function sim_fo(x)
        G = tf([x[1]], [x[2], 1.0])
        res = lsim(G, u_matrix, t)
        # res.y tiene dimensiones (ny, length(t)) → extraer como vector
        ys = vec(res.y)
        return G, ys
    end

    # Optimización: clampar parámetros dentro de la función de costo
    function cost_fo(x)
        alpha_c = clamp(x[1], 0.01, 2500.0)
        tau_c   = clamp(x[2], 0.01, 1.0)
        try
            _, ys = sim_fo([alpha_c, tau_c])
            return norm(ym .- ys)
        catch
            return 1e12  # penalizar si la simulación falla
        end
    end

    result = Optim.optimize(cost_fo, [alpha0, 0.25], NelderMead())
    xopt = Optim.minimizer(result)
    alpha = clamp(xopt[1], 0.01, 2500.0)
    tau   = clamp(xopt[2], 0.01, 1.0)
    G1, ysim1 = sim_fo([alpha, tau])
    r1 = 100.0 * (1.0 - norm(ym .- ysim1) / norm(ym))

    # ── Gráfica comparativa ──────────────────────────────────────────
    modelstr1 = @sprintf("G₁(s) = %.3f/(%.3f·s+1)  FIT=%.1f%%", alpha, tau, r1)

    xlims_v = (t[end] - 20, t[end])
    plt = plot(layout=(2, 1), size=(900, 550),
        title=["Modelos PRBS  yOP=$(@sprintf("%.0f",yop)) °/s" ""],
        ylabel=["Velocidad (°/s)" "Voltaje (V)"],
        xlabel=["Tiempo (s)" "Tiempo (s)"],
        xlims=[xlims_v xlims_v],
        background_color_subplot=[:ivory :mintcream],
        legend=:bottomright, grid=true, gridalpha=0.15, margin=5Plots.mm)
    plot!(plt, subplot=1, t, ym .+ ymean_val,
          label="Datos", color=:teal, linewidth=1.5, linestyle=:dot)
    plot!(plt, subplot=1, t, ysim1 .+ ymean_val,
          label=modelstr1, color=:deeppink, linewidth=1.5)
    plot!(plt, subplot=2, t, u,
          label="PRBS", color=:green)
    display(plt)

    for path in (PATH_DATA, PATH_DEFAULT)
        open(path * "DCmotor_fo_model_pbrs.csv", "w") do io
            println(io, "alpha,tau")
            @printf(io, "%.8f,%.8f\n", alpha, tau)
        end
    end

    return G1
end
