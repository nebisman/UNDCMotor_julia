#import Pkg; Pkg.add("DSP")
using Statistics, DSP, Plots, Printf
using DelimitedFiles
using ControlSystems
using ControlSystemsBase
using LinearAlgebra
using Polynomials
using Alert
using InvertedIndices


function read_csv_file3(filepath::String)
    data = readdlm(filepath, ','; header=true)[1]
    return Float64.(data[:, 1]), Float64.(data[:, 2]), Float64.(data[:, 3])
end
const PATH_DEFAULT = "/home/leonardo/datos/share_desktop/proyecto_julia/motor/julia_code/UNDCMotor/experiment_files/"



"""
Estructura con los resultados de stepinfo para datos experimentales reales.
"""
struct StepInfoSR
    y0::Float64
    yf::Float64
    stepsize::Float64
    peak::Float64
    peaktime::Float64        # relativo al instante del escalón
    overshoot::Float64
    lowerpeak::Float64
    undershoot::Float64
    settlingtime::Float64    # relativo al instante del escalón
    risetime::Float64
    settling_th::Float64
    risetime_th::Tuple{Float64, Float64}
    u_max::Float64          #  maximo de  la señal de control 
end

"""
    stepinfo_real(t, u, y; settling_th=0.02, risetime_th=(0.1, 0.9))

Calcula métricas de respuesta al escalón a partir de datos experimentales reales.
Detecta automáticamente el instante del escalón y mide tiempos desde allí.

# Argumentos
- `t`: vector de tiempos equiespaciados
- `u`: vector de entrada (escalón)
- `y`: vector de salida medida
- `settling_th`: umbral para tiempo de estabilización (default 0.02 = 2%)
- `risetime_th`: umbrales para tiempo de subida (default 10% y 90%)
"""
function stepinfo_exp(res; T = nothing,  settling_th = 0.02, risetime_th = (0.1, 0.9))
    
    t = res[1] 
    r = res[2]
    y = res[3]
    u = res[4]

    u_max = maximum(abs.(u))

    n  = length(y)
    Ts = t[2] - t[1]
    fs = 1.0 / Ts

    # ── Detectar instante del escalón en u ────────────────────────────
    step_idx = argmax(abs.(diff(r)))
    t_step   = t[step_idx]
    t= t.-t_step
    # ── Valores de estado estacionario ────────────────────────────────
    #  y0: promedio de hasta 50 puntos ANTES del escalón
    #  yf: promedio de los últimos 50 puntos
    n_pre = min(step_idx, 50)
    y0    = mean(y[step_idx - n_pre + 1 : step_idx])
    n_end = min(50, n)
    yf    = mean(y[n - n_end + 1 : n])

    # ── Dirección y magnitud (desde u) ────────────────────────────────
    
    rf = r[end]
    r0 = r[1]
    stepsize  = abs(rf- r0)


    # ── Sobrepico (overshoot) — solo post-escalón ─────────────────────
    peak, pidx = findmax(y) 
    peaktime   = t[pidx] 
    overshoot  = 100.0 * (peak - yf) / stepsize

    # ── Subpico (undershoot) — solo post-escalón ──────────────────────
    lowerpeak, _ =  findmin(y) 
    undershoot   = max(100.0 * (y0 - lowerpeak) / stepsize, 0.0)

    # ── Tiempo de estabilización ──────────────────────────────────────
    #  Butterworth causal orden 4, fc=1 Hz sobre y_post reflejada.
    #  El filtro ve primero el estado estacionario; el primer índice
    #  donde la salida se sale de la banda marca el settling time.
    
    responsetype = Lowpass(12)
    designmethod = FIRWindow(hanning(32))
    
   
    settle_filt    = digitalfilter(responsetype, designmethod; fs=fs)
    y_filtered = filtfilt(settle_filt, y)
    band           = settling_th * stepsize
    
    idx_rev        = findfirst(abs.(reverse(y_filtered) .- yf) .> band)
    t_rev    = reverse(t)
     

    if idx_rev === nothing
        settlingtime = NaN
    else
        settlingtime = t_rev[idx_rev] 
    end
 

    # ── Tiempo de subida — interpolación suave post-escalón ───────────
    #  filtfilt (cero fase) Butterworth orden 2, fc=5 Hz.
    #  Solo primer cruce para evitar efecto del ruido.
  
    
   

    lv10 = y0 + risetime_th[1] * stepsize 
    lv90 = y0 + risetime_th[2] * stepsize 
    op   =  (>)
    i10  = findfirst(op.(y_filtered, lv10))
    i90  = findfirst(op.(y_filtered, lv90))

    if i10 === nothing || i90 === nothing
        risetime = NaN
    else
        t10      = _interp_cross(t, y_filtered, lv10, i10)
        t90      = _interp_cross(t, y_filtered, lv90, i90)
        risetime = t90 - t10
    end

    info = StepInfoSR(y0, yf, stepsize, peak, peaktime, overshoot,
                      lowerpeak, undershoot, settlingtime, risetime,
                      settling_th, risetime_th,  u_max)

    _plot_stepinfo(t, r, y_filtered, t10, t90,i10, i90,  info, T)
    return info
end

# ── Interpolación lineal para cruce preciso ───────────────────────────
function _interp_cross(t, y, level, idx)
    idx == 1 && return t[1]
    α = (level - y[idx-1]) / (y[idx] - y[idx-1])
    return t[idx-1] + α * (t[idx] - t[idx-1])
end

# ── Gráfica con anotaciones ──────────────────────────────────────────
function _plot_stepinfo(t, r, y, t10, t90, i10, i90,  si, T)

    
    

    p =    plot( t, r;
        label = "r(t)", lw = 1.5, color = :green, alpha=0.7, background_color=:mintcream)

    if  T !== nothing
        t1 = 0: 1/1000: t[end]
        escalon(x, x0) = x >= x0 ? r[end] : r[1]
        r1 = escalon.(t1, 0)      
        res = lsim(T, r1', t1)
        plot!(p, res,  color = "#00aad4", alpha=0.5, seriestype=:steppost,
        label = "y(t) simulada")   
    end      
    plot!(t, y;
        label  = "y(t) medida (yf = $(@sprintf("%.2f",si.yf)))",
        lw     = 2, color = :navy, alpha = 1,
        xlabel = "Tiempo (s)", ylabel = "Amplitud",
        title  = "Respuesta experimental al Escalón",
        legend = :right, size = (900, 500),        
        )

    
    # Banda de estabilización
    hline!(p, [si.yf + si.settling_th * si.stepsize,
               si.yf - si.settling_th * si.stepsize];
        ls = :dot, color = :orange, alpha = 0.5, label ="" )

    # Pico (sobrepico) — posición absoluta
    scatter!(p, [si.peaktime], [si.peak];
        ms = 5, color = :red, markershape = :circle,
        label = "Pico = $(@sprintf("%.2f",si.peak)) (SP = $(@sprintf("%.2f",si.overshoot)))")

    # Tiempo de estabilización — posición absoluta
    vline!(p, [si.settlingtime];
        ls = :dashdot, color = :orange, lw = 1.25,
        label = "Ts = $(@sprintf("%.2f",si.settlingtime)) s")

    # Niveles 10% y 90% para tiempo de subida
    if !isnan(si.risetime)
        lv10 = si.y0 + si.risetime_th[1] * si.stepsize
        lv90 = si.y0 + si.risetime_th[2] * si.stepsize
        vline!(p, [t10, t90];
            ls = :dashdot, color = :purple, alpha = 0.4, lw=1.25, label = "")
        annotate!(p, t10 + 0.4*si.risetime , lv10, text("10%", 7, :purple))
        annotate!(p, t90 - 0.4*si.risetime, lv90, text("90%", 7, :purple))
        #plot!(t[i10:i90], y[i10:i90], color = "#800033", label ="")
    end

    # Leyenda informativa
    plot!(p, Float64[], Float64[];
        label = "Tr = $(@sprintf("%.2f",si.risetime)) s", lw = 1,
        ls = :dashdot, color = :purple, alpha = 0.6)
        
    # plot!(p, Float64[], Float64[];
    #     label = "Subpico = $(round(si.undershoot; digits=2))%", lw = 0)
            

    display(p)
end

# ── Mostrar resumen en consola ────────────────────────────────────────
function Base.show(io::IO, si::StepInfoSR)
    println(io, "StepInfoSR:")
    @printf(io, "  %-18s %8.3f\n",   "Valor inicial:",    si.y0)
    @printf(io, "  %-18s %8.3f\n",   "Valor final:",      si.yf)
    @printf(io, "  %-18s %8.3f\n",   "Cambio escalón:",   si.stepsize)
    @printf(io, "  %-18s %8.3f\n",   "Pico:",             si.peak)
    @printf(io, "  %-18s %8.3f s\n", "Tiempo pico:",      si.peaktime)
    @printf(io, "  %-18s %8.2f %%\n","Sobrepico:",        si.overshoot)
    @printf(io, "  %-18s %8.3f s\n", "T. estabilización:",    si.settlingtime)
    @printf(io, "  %-18s %8.3f s\n", "T. subida:",        si.risetime)
    @printf(io, "  %-18s %8.3f V\n", "Max. u(t):",        si.u_max)

end



