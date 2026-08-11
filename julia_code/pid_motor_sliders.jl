### A Pluto.jl notebook ###
# v1.0.1

using Markdown
using InteractiveUtils

# This Pluto notebook uses @bind for interactivity. When running this notebook outside of Pluto, the following 'mock version' of @bind gives bound variables a default value (instead of an error).
macro bind(def, element)
    #! format: off
    return quote
        local iv = try Base.loaded_modules[Base.PkgId(Base.UUID("6e696c72-6542-2067-7265-42206c756150"), "AbstractPlutoDingetjes")].Bonds.initial_value catch; b -> missing; end
        local el = $(esc(element))
        global $(esc(def)) = Core.applicable(Base.get, el) ? Base.get(el) : iv(el)
        el
    end
    #! format: on
end

# ╔═╡ 11111111-0000-0000-0000-000000000001
begin
    using Pkg
    Pkg.activate()
    using PlutoUI
    using ControlSystems
    using Plots
    using Printf
    using LaTeXStrings
    using UNDCMotor
    # ── Cargar módulo UNDCMotor ──────────────────────────────────────────────
    include("UNDCMotor.jl")
    
end

# ╔═╡ 22222222-0000-0000-0000-000000000001
md"""
# 🎛️ Diseño PID para Motor DC — Control en tiempo real

Ajusta los parámetros de diseño con los sliders y presiona **▶ Ejecutar experimento** para
cargar el PID al ESP32 y ver la respuesta en tiempo real.

---
"""

# ╔═╡ 33333333-0000-0000-0000-000000000001
md"### Parámetros de diseño"

# ╔═╡ 44444444-0000-0000-0000-000000000001
@bind ωn Slider(1.0:0.5:20.0, default=9.0, show_value=true)

# ╔═╡ 44444444-0000-0000-0000-000000000002
md"**ωₙ** (frecuencia natural deseada, rad/s)"

# ╔═╡ 55555555-0000-0000-0000-000000000001
@bind ζ Slider(0.1:0.05:1.5, default=0.7, show_value=true)

# ╔═╡ 55555555-0000-0000-0000-000000000002
md"**ζ** (coeficiente de amortiguamiento)"

# ╔═╡ 66666666-0000-0000-0000-000000000001
@bind n_val Slider(1:1:5, default=2, show_value=true)

# ╔═╡ 66666666-0000-0000-0000-000000000002
md"**n** (factor de separación de polos del tercer polo)"

# ╔═╡ 77777777-0000-0000-0000-000000000001
@bind metodo Select(["2do orden" => "2do orden", "ITAE" => "ITAE"], default="ITAE")

# ╔═╡ 77777777-0000-0000-0000-000000000002
md"**Método de diseño**"

# ╔═╡ 88888888-0000-0000-0000-000000000001
# ── Planta nominal ───────────────────────────────────────────────────────────
begin
    _sys_nominal = MotorSystem(port="/dev/ttyUSB0")
    G_ang = transfer_function(_sys_nominal)
    b_plant = numvec(G_ang)[1][1]
    a_plant = denvec(G_ang)[1][2]
    G_ang
end

# ╔═╡ 99999999-0000-0000-0000-000000000001
# ── Cálculo reactivo de constantes PID ───────────────────────────────────────
begin
    s = tf("s")

    local Kp_val, Ki_val, Kd_val, T_cl

    if metodo == "2do orden"
        # Método 2do orden con tercer polo en n·ωn
        T_cl  = n_val * ωn^3 / ((s + n_val*ωn) * (s^2 + 2*ζ*ωn*s + ωn^2))
        Kd_val = ((2ζ + n_val) * ωn - a_plant) / b_plant
        Kp_val = (1 + 2*n_val*ζ) * ωn^2 / b_plant
        Ki_val = n_val * ωn^3 / b_plant
    else
        # Método ITAE
        T_cl  = ωn^3 / (s^3 + 1.75*ωn*s^2 + 2.15*ωn^2*s + ωn^3)
        Kd_val = (1.75 * ωn - a_plant) / b_plant
        Kp_val = 2.15 * ωn^2 / b_plant
        Ki_val = ωn^3 / b_plant
    end

    # Tabla de resultados
    md"""
    ### 📐 Constantes PID calculadas

    | Parámetro | Valor |
    |-----------|-------|
    | **Kp**    | $(round(Kp_val, digits=4)) |
    | **Ki**    | $(round(Ki_val, digits=4)) |
    | **Kd**    | $(round(Kd_val, digits=4)) |
    | Método    | $metodo |
    | ωₙ        | $ωn rad/s |
    | ζ         | $ζ |
    | n         | $(metodo == "2do orden" ? n_val : "N/A (ITAE)") |
    """
end

# ╔═╡ aaaaaaaa-0000-0000-0000-000000000001
# ── Respuesta simulada en lazo cerrado ───────────────────────────────────────
begin
    t_sim = 0:0.001:3.0
    r_sim = 100.0 .* ones(length(t_sim))

    res_sim = lsim(T_cl, r_sim', collect(t_sim))
    y_sim_vec = vec(res_sim.y)

    p_preview = plot(
        collect(t_sim), [r_sim, y_sim_vec];
        label      = ["r(t) = 100°" "y(t) simulada"],
        color      = [:teal :deeppink],
        linewidth  = [1.2 2.0],
        linestyle  = [:dash :solid],
        xlabel     = "Tiempo (s)",
        ylabel     = "Ángulo (°)",
        title      = "Simulación lazo cerrado — ωₙ=$ωn rad/s, ζ=$ζ, $metodo",
        legend     = :right,
        background_color = :mintcream,
        size       = (820, 350),
        grid       = true, gridalpha = 0.2,
    )

    # Calcular métricas aproximadas desde simulación
    yf_s  = y_sim_vec[end]
    band2 = 0.02 * 100.0
    idx_ts = findlast(abs.(y_sim_vec .- yf_s) .> band2)
    ts_s  = idx_ts === nothing ? 0.0 : t_sim[idx_ts]
    pk_s, pidx_s = findmax(y_sim_vec)
    os_s  = 100.0 * (pk_s - yf_s) / 100.0

    annotate!(p_preview,
        t_sim[end]*0.6, 10.0,
        text("Ts ≈ $(round(ts_s, digits=2)) s   OS ≈ $(round(os_s, digits=1)) %", 9, :gray40)
    )
    p_preview
end

# ╔═╡ bbbbbbbb-0000-0000-0000-000000000001
md"""
---
### ⚡ Experimento en hardware

Configura el escalón y presiona el botón para ejecutar en el ESP32.
"""

# ╔═╡ bbbbbbbb-0000-0000-0000-000000000002
@bind r_final Slider(10.0:10.0:360.0, default=100.0, show_value=true)

# ╔═╡ bbbbbbbb-0000-0000-0000-000000000003
md"**Referencia final r₁** (grados)"

# ╔═╡ bbbbbbbb-0000-0000-0000-000000000004
@bind t_exp_high Slider(0.5:0.5:5.0, default=3.0, show_value=true)

# ╔═╡ bbbbbbbb-0000-0000-0000-000000000005
md"**Duración del escalón** (segundos)"

# ╔═╡ cccccccc-0000-0000-0000-000000000001
@bind beta_val Slider(0.0:0.1:1.0, default=0.0, show_value=true)

# ╔═╡ cccccccc-0000-0000-0000-000000000002
md"**β** (ponderación de referencia en PID 2DOF, 0 = 1DOF)"

# ╔═╡ dddddddd-0000-0000-0000-000000000001
@bind run_btn Button("▶  Ejecutar experimento en ESP32")

# ╔═╡ eeeeeeee-0000-0000-0000-000000000001
# ── Celda de ejecución reactiva al botón ─────────────────────────────────────
# Estrategia para tiempo real en Pluto:
#   1. Se crea un Canal compartido entre el callback on_frame y esta celda.
#   2. step_closed llama on_frame en cada trama recibida del ESP32 y
#      deposita los datos en el canal.
#   3. Esta celda consume el canal en un bucle, actualizando la figura con
#      Plots y usando yield() / sleep() para ceder el hilo a Pluto y que
#      la UI se refresque en cada iteración.
#   4. El patrón @htl + PlutoUI.combine no es necesario porque Plots.jl
#      emite SVG/PNG que Pluto renderiza directamente al hacer `display()`.
begin
    run_btn   # reactiva esta celda cada vez que se presiona el botón

    # Capturar valores actuales de los sliders (immutable en esta ejecución)
    local _ωn  = ωn
    local _ζ   = ζ
    local _n   = n_val
    local _met = metodo
    local _r1  = r_final
    local _th  = t_exp_high
    local _β   = beta_val

    # ── Recalcular ganancias con valores capturados ──────────────────────────
    local _s = tf("s")
    local _Kp, _Ki, _Kd, _T

    if _met == "2do orden"
        _T  = _n * _ωn^3 / ((_s + _n*_ωn) * (_s^2 + 2*_ζ*_ωn*_s + _ωn^2))
        _Kd = ((2_ζ + _n) * _ωn - a_plant) / b_plant
        _Kp = (1 + 2*_n*_ζ) * _ωn^2 / b_plant
        _Ki = _n * _ωn^3 / b_plant
    else
        _T  = _ωn^3 / (_s^3 + 1.75*_ωn*_s^2 + 2.15*_ωn^2*_s + _ωn^3)
        _Kd = (1.75 * _ωn - a_plant) / b_plant
        _Kp = 2.15 * _ωn^2 / b_plant
        _Ki = _ωn^3 / b_plant
    end

    @info "Cargando PID → Kp=$(_Kp) Ki=$(_Ki) Kd=$(_Kd) β=$(_β)"

    # ── Canal para streaming de datos entre hilos ────────────────────────────
    local ch = Channel{NamedTuple}(256)

    # ── Constantes del protocolo ─────────────────────────────────────────────
    local h   = SAMPLING_TIME
    local bs  = BUFFER_SIZE
    local t0  = 1.0                     # pre-escalón (1 segundo en cero)
    local t1  = _th
    local delta = abs(_r1)

    # Vectores acumuladores (compartidos entre hilo lector y hilo gráfico)
    local tv_acc = Float64[]
    local rv_acc = Float64[]
    local yv_acc = Float64[]
    local uv_acc = Float64[]

    # ── Figura inicial vacía ─────────────────────────────────────────────────
    local fig_rt = plot(
        layout = (2, 1), size = (860, 480),
        title  = ["Step cerrado — ωₙ=$(_ωn), ζ=$(_ζ), $(_met)" ""],
        ylabel = ["Ángulo (°)" "Control (V)"],
        xlabel = ["Tiempo (s)" "Tiempo (s)"],
        xlims  = [(0, t0 + t1) (0, t0 + t1)],
        ylims  = [(-0.3*delta, _r1 + 0.6*delta) (-5.5, 5.5)],
        background_color_subplot = [:ivory :mintcream],
        legend = :bottomright, grid = true, gridalpha = 0.2,
        margin = 5Plots.mm,
    )
    display(fig_rt)

    # ── Tarea de comunicación con el ESP32 (hilo separado) ───────────────────
    local reader_task = @async begin
        local _sys = MotorSystem(port="/dev/ttyUSB0")
        try
            # 1. Cargar PID
            set_pid(_sys;
                kp       = _Kp,
                ki       = _Ki,
                kd       = _Kd,
                beta     = _β,
                output   = :angle,
                deadzone = 0.2,
            )
            sleep(0.3)

            # 2. Ejecutar escalón: reutilizamos la lógica de step_closed
            #    pero redirigimos cada trama al canal `ch` en lugar de
            #    llamar a display() directamente (lo que bloquearía el hilo).
            local pts_low  = round(Int, t0 / h)
            local pts_high = round(Int, t1 / h)
            local total    = pts_low + pts_high
            local frames   = ceil(Int, total / bs)

            local payload = Dict(
                "low_val"     => float2hex(0.0),
                "high_val"    => float2hex(Float64(_r1)),
                "points_low"  => long2hex(pts_low),
                "points_high" => long2hex(pts_high),
            )

            connect!(_sys)
            send_command!(_sys, "step_closed", payload)

            local on_frame = function(msg, frame_no)
                local rf = hexframe_to_array(string(msg["r"]))
                local uf = hexframe_to_array(string(msg["u"]))
                local yf = hexframe_to_array(string(msg["y"]))
                local tf = h .* (collect(0:length(rf)-1) .+ (frame_no - 1) * bs)
                put!(ch, (t=tf, r=rf, y=yf, u=uf))
            end

            receive_frames!(_sys, frames, on_frame; timeout_factor=2.0)

        catch e
            @warn "Error en experimento: $e"
        finally
            try disconnect!(_sys) catch end
            close(ch)   # señal de fin al bucle gráfico
        end
    end

    # ── Bucle gráfico: consume el canal y actualiza la figura ────────────────
    # Este bucle corre en el hilo principal de la celda Pluto.
    # yield() entre iteraciones permite que el servidor Pluto envíe la
    # actualización al navegador (WebSocket) sin bloquear.
    local t_total_exp = t0 + t1

    for frame_data in ch
        append!(tv_acc, frame_data.t)
        append!(rv_acc, frame_data.r)
        append!(yv_acc, frame_data.y)
        append!(uv_acc, frame_data.u)

        # Reconstruir figura desde cero (evita trazas duplicadas)
        fig_rt = plot(
            layout = (2, 1), size = (860, 480),
            title  = ["Step cerrado — ωₙ=$(_ωn), ζ=$(_ζ), $(_met)" ""],
            ylabel = ["Ángulo (°)" "Control (V)"],
            xlabel = ["Tiempo (s)" "Tiempo (s)"],
            xlims  = [(0, t_total_exp) (0, t_total_exp)],
            ylims  = [(-0.3*delta, _r1 + 0.6*delta) (-5.5, 5.5)],
            background_color_subplot = [:ivory :mintcream],
            legend = :bottomright, grid = true, gridalpha = 0.15,
            margin = 5Plots.mm,
        )
        plot!(fig_rt, subplot=1,
              tv_acc, rv_acc;
              label="r(t)", color=:teal, linewidth=1.2, seriestype=:steppost)
        plot!(fig_rt, subplot=1,
              tv_acc, yv_acc;
              label="y(t)", color=:deeppink, linewidth=1.5)
        plot!(fig_rt, subplot=2,
              tv_acc, uv_acc;
              label="u(t)", color=:royalblue, linewidth=1.2)

        display(fig_rt)
        yield()          # ← cede el hilo → Pluto envía el SVG actualizado al browser
    end

    # ── Resultado final: stepinfo ─────────────────────────────────────────────
    wait(reader_task)    # asegurar que el hilo de comunicación terminó

    if length(tv_acc) > 10
        result_tuple = (tv_acc, rv_acc, yv_acc, uv_acc)
        @info "Experimento completado — $(length(tv_acc)) muestras"
        stepinfo_exp(result_tuple; T = _T)
    end

    fig_rt   # muestra la figura final en la celda
end

# ╔═╡ ffffffff-0000-0000-0000-000000000001
md"""
---
### 📋 Notas técnicas

**¿Por qué `yield()` en el bucle gráfico?**

En Pluto, cada celda se ejecuta en una tarea Julia (`@async`). El servidor Pluto
actualiza el navegador (vía WebSocket) entre tareas. Si el bucle nunca cede el
hilo (`yield()`), la figura no se transmite hasta que termina todo el experimento.
Con `yield()` al final de cada iteración, Pluto envía el SVG actualizado al browser
en cada trama recibida del ESP32, logrando la actualización en tiempo real.

**Arquitectura de dos hilos**

| Hilo | Responsabilidad |
|------|----------------|
| `@async reader_task` | Comunicación serial con ESP32 · publica en `Channel` |
| Bucle principal celda | Consume `Channel` · actualiza figura · llama `yield()` |

El `Channel` actúa como buffer desacoplador: si el hilo de comunicación produce
datos más rápido de lo que el hilo gráfico los consume, los datos se acumulan en
el canal sin pérdida (capacidad 256 tramas).
"""

# ╔═╡ Cell order:
# ╠═11111111-0000-0000-0000-000000000001
# ╟─22222222-0000-0000-0000-000000000001
# ╟─33333333-0000-0000-0000-000000000001
# ╠═44444444-0000-0000-0000-000000000001
# ╟─44444444-0000-0000-0000-000000000002
# ╠═55555555-0000-0000-0000-000000000001
# ╟─55555555-0000-0000-0000-000000000002
# ╠═66666666-0000-0000-0000-000000000001
# ╟─66666666-0000-0000-0000-000000000002
# ╠═77777777-0000-0000-0000-000000000001
# ╟─77777777-0000-0000-0000-000000000002
# ╠═88888888-0000-0000-0000-000000000001
# ╠═99999999-0000-0000-0000-000000000001
# ╠═aaaaaaaa-0000-0000-0000-000000000001
# ╟─bbbbbbbb-0000-0000-0000-000000000001
# ╠═bbbbbbbb-0000-0000-0000-000000000002
# ╟─bbbbbbbb-0000-0000-0000-000000000003
# ╠═bbbbbbbb-0000-0000-0000-000000000004
# ╟─bbbbbbbb-0000-0000-0000-000000000005
# ╠═cccccccc-0000-0000-0000-000000000001
# ╟─cccccccc-0000-0000-0000-000000000002
# ╠═dddddddd-0000-0000-0000-000000000001
# ╠═eeeeeeee-0000-0000-0000-000000000001
# ╟─ffffffff-0000-0000-0000-000000000001
