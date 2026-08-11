# =============================================================================
#  Motor DC: datos sintéticos y gráfica en tiempo real  —  Jupyter / IJulia
#
#      G(s) = Ω(s)/V(s) = 3369 / (s + 19.58)      [(grados/s) / voltio]
#
#  Tramas de 50 muestras con Ts = 10 ms (una trama cada 0.5 s); la figura se
#  refresca EN SITIO, sin apilar una gráfica nueva por cada trama.
#
#  Uso:
#      include("motor_dc_jupyter.jl")
#      diagnostico_jupyter()      # revisa el entorno
#      prueba_refresco()          # 10 figuras: debe verse UNA sola actualizándose
#      e = simular_motor_dc(4.0, 5.0)
#      resumen(e)
#
#  ---------------------------------------------------------------------------
#  CÓMO SE EVITA EL APILAMIENTO
#
#  En Jupyter cada `display` agrega una salida nueva a la celda; el backend
#  (GR, PlotlyJS, PGFPlotsX) es irrelevante, el apilamiento ocurre en el
#  protocolo, no en el dibujado. Hay dos mecanismos para evitarlo:
#
#   1. update_display_data  (el que usa este archivo por defecto)
#      La figura se publica UNA vez con un `display_id`; los refrescos
#      posteriores envían "update_display_data" con ese mismo id y el frontend
#      REEMPLAZA ese objeto de salida. Nunca apila y no parpadea, porque no
#      hay borrado de por medio. Es lo mismo que display(obj, display_id=...)
#      en Python.
#
#   2. IJulia.clear_output(true) + display   (respaldo automático)
#      Borra la salida anterior. Es el método documentado por IJulia, pero
#      depende de que el frontend honre el borrado diferido, cosa que algunos
#      clientes (VS Code, ciertas versiones de JupyterLab) no hacen bien: ahí
#      es donde se ven las gráficas apiladas.
#
#  Además, NADA de `println` dentro del bucle: el stdout de IJulia se vacía de
#  forma asíncrona, el texto llega después del refresco y se acumula. El avance
#  de la adquisición va en el título de la figura.
# =============================================================================

using Plots
using Random
using Printf

gr()   # PNG liviano: el backend más rápido para refrescar en el notebook

# =============================================================================
#  A. Refresco en sitio de la salida de la celda
# =============================================================================

"Devuelve el módulo IJulia si el kernel lo cargó, o `nothing` fuera de Jupyter."
_ijulia() = isdefined(Main, :IJulia) ? getfield(Main, :IJulia) : nothing

"El socket de publicación cambió de tipo entre versiones de IJulia."
function _socket_pub(IJ)
    pub = getfield(IJ, :publish)
    return pub isa Base.RefValue ? pub[] : pub
end

"""
    _publicar(plt, id, actualizar)

Envía la figura por el canal IOPub con un `display_id`. Si `actualizar` es
`false` crea el objeto de salida ("display_data"); si es `true` reemplaza el
que ya existe ("update_display_data"), que es lo que evita el apilamiento.
"""
function _publicar(plt, id::String, actualizar::Bool)
    IJ = _ijulia()
    contenido = Dict("data"      => IJ.display_dict(plt),
                     "metadata"  => Dict(),
                     "transient" => Dict("display_id" => id))
    tipo = actualizar ? "update_display_data" : "display_data"
    IJ.send_ipython(_socket_pub(IJ), IJ.msg_pub(IJ.execute_msg, tipo, contenido))
    return nothing
end

"""
    Lienzo(; modo = :auto)

Área de salida reutilizable. `modo` puede ser:
- `:auto`         -> intenta `update_display_data` y cae a `clear_output` si falla
- `:display_id`   -> fuerza `update_display_data`
- `:clear_output` -> fuerza `IJulia.clear_output(true)` + `display`
- `:simple`       -> `display` a secas (REPL, o para comprobar el apilamiento)
"""
mutable struct Lienzo
    id::String
    modo::Symbol
    publicado::Bool
end

function Lienzo(; modo::Symbol = :auto)
    id = string("motor-dc-", string(rand(UInt64), base = 16))   # id nuevo por corrida
    if modo === :auto
        modo = _ijulia() === nothing ? :simple : :display_id
    end
    return Lienzo(id, modo, false)
end

"""
    refrescar!(l::Lienzo, plt)

Muestra `plt` reemplazando la figura anterior. Si el camino preferido falla
(por ejemplo porque cambiaron los internos de IJulia), degrada de forma
automática al siguiente mecanismo y avisa una sola vez.
"""
function refrescar!(l::Lienzo, plt)
    IJ = _ijulia()

    if l.modo === :display_id
        try
            _publicar(plt, l.id, l.publicado)
            l.publicado = true
            return nothing
        catch err
            @warn "update_display_data no disponible; se usa clear_output." err
            l.modo = :clear_output
        end
    end

    if l.modo === :clear_output && IJ !== nothing
        flush(stdout)                       # evita texto pendiente tras el borrado
        try
            IJ.clear_output(true)
        catch err
            @warn "clear_output falló; la salida quedará apilada." err
            l.modo = :simple
        end
    end

    display(plt)
    return nothing
end

# =============================================================================
#  B. Diagnóstico y prueba mínima
# =============================================================================

"""
    diagnostico_entorno()

Identifica dónde se está ejecutando y qué mecanismo de refresco hay disponible.
Ejecútelo primero si las gráficas se siguen apilando: la pila de displays dice
quién está mostrando las figuras.
"""
function diagnostico_entorno()
    IJ  = _ijulia()
    vsc = isdefined(Main, :VSCodeServer)
    plu = isdefined(Main, :PlutoRunner)

    println("Main.IJulia (kernel IJulia):           ", IJ !== nothing)
    println("Main.VSCodeServer (kernel de VS Code): ", vsc)
    println("Main.PlutoRunner (Pluto):              ", plu)
    println("isinteractive():                       ", isinteractive())
    println("Backend de Plots:                      ", Plots.backend())
    println("Pila de displays (el último manda):")
    for d in Base.Multimedia.displays
        println("    ", typeof(d))
    end
    println()

    if IJ !== nothing
        inited = isdefined(IJ, :inited) ? getfield(IJ, :inited) : "campo ausente"
        println("IJulia.inited: ", inited)
        for f in (:clear_output, :display_dict, :send_ipython, :msg_pub, :publish, :execute_msg)
            println(rpad("    IJulia.$f", 34), isdefined(IJ, f))
        end
        println("\nEntorno: kernel IJulia. Modo de refresco: ", Lienzo().modo)
        println("Verifíquelo con prueba_refresco(10).")
    elseif vsc
        println("""
        Entorno: kernel propio de la extensión de Julia para VS Code (o su REPL),
        NO IJulia. Ese kernel no expone clear_output ni update_display_data, así
        que cada display agrega una salida nueva: de ahí el apilamiento, y ningún
        cambio en este archivo puede evitarlo.

        Solución: use el kernel de IJulia.
            1) En el REPL de Julia:  ] add IJulia
            2) En el notebook: "Select Kernel" -> "Jupyter Kernel..." -> "Julia 1.x"
               (el que instala IJulia, no la entrada del kernel de la extensión)
            3) Vuelva a ejecutar diagnostico_entorno(): Main.IJulia debe dar true

        Alternativa sin VS Code: en el REPL, using IJulia; IJulia.jupyterlab()""")
    elseif plu
        println("""
        Entorno: Pluto. Aquí el refresco no se hace con display sino devolviendo
        la figura desde una celda reactiva disparada por PlutoUI.Clock.""")
    else
        println("""
        Entorno: REPL o script, sin notebook. Las figuras van a la ventana de GR,
        que sí se actualiza en sitio. Si ve gráficas acumuladas en el panel de
        plots de VS Code, es su galería de historial: se navega con las flechas.""")
    end
    return nothing
end

"Alias por compatibilidad."
diagnostico_jupyter() = diagnostico_entorno()

"""
    prueba_refresco(n = 10; modo = :auto, pausa = 0.3)

Dibuja `n` figuras seguidas. Si el mecanismo funciona se ve **una sola**
gráfica cambiando; si se ven `n` gráficas apiladas, ese modo no sirve en su
frontend. Compare `prueba_refresco(5; modo = :display_id)` contra
`prueba_refresco(5; modo = :clear_output)` para saber cuál usar.
"""
function prueba_refresco(n::Int = 10; modo::Symbol = :auto, pausa::Real = 0.3)
    l = Lienzo(; modo)
    x = range(0, 2π; length = 200)
    for i in 1:n
        plt = plot(x, sin.(x .+ i / 2);
                   color = "#00aad4", lw = 2, ylims = (-1.2, 1.2),
                   title = "prueba de refresco — figura $i de $n  (modo: $(l.modo))",
                   titlefont = font(10), legend = false, framestyle = :box,
                   size = (700, 300))
        refrescar!(l, plt)
        sleep(pausa)
    end
    return l.modo
end

# =============================================================================
#  C. 1-3. Modelo y generador de datos sintéticos
# =============================================================================
const K_MOTOR = 3369.0    # numerador de G(s)
const P_MOTOR = 19.58     # polo de G(s)  [rad/s]

ganancia_dc()      = K_MOTOR / P_MOTOR   # 172.06 (grados/s)/V
constante_tiempo() = 1 / P_MOTOR         # 51.1 ms

"Cuantización tipo encoder; `q <= 0` deshabilita el efecto."
_cuantizar(x, q) = q > 0 ? q * round(x / q) : x

"""
    tramas_motor_dc(V, T_sim; Ts, N_trama, σ_ruido, resolucion, tiempo_real, semilla, buffer)

Emula el procesador que muestrea el sensor y transmite los datos en tramas de
`N_trama` muestras. Devuelve un `Channel` que entrega tuplas `(t, ω)`.

Discretización exacta (ZOH) de `G(s)`, válida en los instantes de muestreo:

    ω[k+1] = a·ω[k] + b·V ,   a = e^(-p·Ts),   b = (K/p)·(1 - a)
"""
function tramas_motor_dc(V::Real, T_sim::Real;
                         Ts::Real          = 0.01,
                         N_trama::Int      = 50,
                         σ_ruido::Real     = 6.0,
                         resolucion::Real  = 0.0,
                         tiempo_real::Bool = true,
                         semilla           = 42,
                         buffer::Int       = 256)

    rng     = semilla === nothing ? Random.default_rng() : MersenneTwister(semilla)
    N_total = floor(Int, T_sim / Ts)          # muestras en [0, T_sim)
    a = exp(-P_MOTOR * Ts)
    b = ganancia_dc() * (1 - a)

    Channel{Tuple{Vector{Float64},Vector{Float64}}}(buffer) do canal
        ω = 0.0        # motor en reposo
        k = 0          # índice global de muestra
        while k < N_total
            n = min(N_trama, N_total - k)     # la última trama puede ser parcial
            t_tr = [(k + i - 1) * Ts for i in 1:n]
            ω_tr = Vector{Float64}(undef, n)

            for i in 1:n
                ω_tr[i] = _cuantizar(ω + σ_ruido * randn(rng), resolucion)  # sensor
                ω = a * ω + b * V                                           # modelo
            end

            k += n
            tiempo_real && sleep(n * Ts)      # el procesador tarda n·Ts en llenar la trama
            put!(canal, (t_tr, ω_tr))         # transmisión de la trama
        end
    end
end

# -----------------------------------------------------------------------------
#  Estado de la adquisición: separa la recepción del dibujado
# -----------------------------------------------------------------------------
mutable struct EstadoMotor
    V::Float64
    T_sim::Float64
    Ts::Float64
    N_trama::Int
    ω_ss::Float64                                             # estado estacionario
    t::Vector{Float64}                                        # datos acumulados
    ω::Vector{Float64}
    canal::Channel{Tuple{Vector{Float64},Vector{Float64}}}
    n_tramas::Int
    terminado::Bool
end

function Base.show(io::IO, e::EstadoMotor)
    @printf(io, "EstadoMotor(%.2f V, %.2f s): %d tramas, %d muestras, %s",
            e.V, e.T_sim, e.n_tramas, length(e.t),
            e.terminado ? "adquisición terminada" : "adquisición en curso")
end

"Arranca el productor en una tarea aparte y devuelve el estado sin bloquear."
function iniciar_motor_dc(V::Real = 4.0, T_sim::Real = 5.0;
                          Ts::Real = 0.01, N_trama::Int = 50, kwargs...)
    ω_ss = ganancia_dc() * V
    ω_ss > 800 && @warn @sprintf(
        "La velocidad final (%.1f grados/s) supera el límite de 800 del eje y. Use V ≤ %.2f V.",
        ω_ss, 800 / ganancia_dc())

    canal = tramas_motor_dc(V, T_sim; Ts, N_trama, kwargs...)
    return EstadoMotor(V, T_sim, Ts, N_trama, ω_ss, Float64[], Float64[], canal, 0, false)
end

"""
    actualizar!(e::EstadoMotor)

Vacía sin bloquear las tramas que ya llegaron y marca `e.terminado` cuando el
productor cerró el canal, es decir cuando no se reciben más datos sintéticos.
"""
function actualizar!(e::EstadoMotor)
    while isready(e.canal)
        t_tr, ω_tr = take!(e.canal)
        append!(e.t, t_tr)
        append!(e.ω, ω_tr)
        e.n_tramas += 1
    end
    if !isopen(e.canal) && !isready(e.canal)
        e.terminado = true
    end
    return e
end

"Cierra la adquisición antes de tiempo (por ejemplo al re-ejecutar celdas)."
function detener!(e::EstadoMotor)
    isopen(e.canal) && close(e.canal)
    e.terminado = true
    return e
end

# -----------------------------------------------------------------------------
#  4, 7, 8. Figura
# -----------------------------------------------------------------------------
function graficar(e::EstadoMotor; kwargs...)
    t_actual = isempty(e.t) ? 0.0 : last(e.t)

    plt = plot(e.t, e.ω;
        xlims      = (0, e.T_sim),            # límite en x = tiempo de simulación
        ylims      = (0, 800),                # límite en y = 0 a 800 grados/s
        xlabel     = "Tiempo [s]",
        ylabel     = "Velocidad angular [grados/s]",
        title      = @sprintf("Motor DC — %.2f V   |   t = %.2f s de %.2f s   |   trama %d",
                              e.V, t_actual, e.T_sim, e.n_tramas),
        titlefont  = font(10),
        color      = "#00aad4",
        lw         = 2,
        label      = "ω medida (sensor)",
        legend     = :bottomright,
        framestyle = :box,
        grid       = true,
        size       = (800, 450),
        kwargs...)

    hline!(plt, [e.ω_ss]; color = :gray40, ls = :dash, lw = 1,
           label = @sprintf("ω_ss = %.1f grados/s", e.ω_ss))
    return plt
end

# -----------------------------------------------------------------------------
#  5-6. Bucle de tiempo real
# -----------------------------------------------------------------------------
"""
    simular_motor_dc(V=4.0, T_sim=5.0; modo=:auto, gif_salida=nothing, kwargs...) -> EstadoMotor

Recibe una trama cada 0.5 s y actualiza la gráfica **en la misma salida** hasta
terminar `T_sim`. Los datos quedan en `e.t` y `e.ω`.

`modo` selecciona el mecanismo de refresco (ver [`Lienzo`](@ref)); con `:auto`
usa `update_display_data` dentro de Jupyter.

Ejemplos
```julia
e = simular_motor_dc()                               # 4 V durante 5 s
e = simular_motor_dc(2.0, 8.0)                       # otro voltaje y duración
e = simular_motor_dc(4.0, 5.0; σ_ruido = 15.0)       # sensor más ruidoso
e = simular_motor_dc(4.0, 5.0; resolucion = 5.0)     # cuantización tipo encoder
e = simular_motor_dc(4.0, 5.0; modo = :clear_output) # mecanismo alterno
e = simular_motor_dc(4.0, 5.0; gif_salida = "motor_dc.gif")
```
"""
function simular_motor_dc(V::Real = 4.0, T_sim::Real = 5.0;
                          modo::Symbol = :auto,
                          gif_salida   = nothing,
                          kwargs...)

    e      = iniciar_motor_dc(V, T_sim; kwargs...)
    lienzo = Lienzo(; modo)
    anim   = gif_salida === nothing ? nothing : Animation()

    refrescar!(lienzo, graficar(e))            # ejes ya fijados, aún sin datos

    while !e.terminado
        n0 = e.n_tramas
        actualizar!(e)

        if e.n_tramas > n0                     # sólo se redibuja si llegaron tramas
            plt = graficar(e)
            refrescar!(lienzo, plt)
            anim === nothing || frame(anim, plt)
        end

        sleep(0.05)                            # sondeo del canal
    end

    anim === nothing || gif(anim, gif_salida; fps = 4)
    return e
end

"""
    resumen(e::EstadoMotor)

Resumen de la adquisición y estimación de la constante de tiempo a partir de
los datos (instante en que se alcanza el 63.2 % de ω_ss).
"""
function resumen(e::EstadoMotor)
    @printf("Muestras: %d   |   tramas: %d   |   ω final: %.1f grados/s   |   ω_ss teórica: %.1f grados/s\n",
            length(e.t), e.n_tramas, last(e.ω), e.ω_ss)
    k63 = findfirst(≥(0.632 * e.ω_ss), e.ω)
    k63 === nothing && return nothing
    @printf("τ teórica: %.1f ms   |   τ estimada de los datos: %.1f ms\n",
            constante_tiempo() * 1e3, e.t[k63] * 1e3)
    return nothing
end