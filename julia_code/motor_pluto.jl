# =============================================================================
#  Motor DC: datos sintéticos y gráfica en tiempo real  —  Pluto, SIN Clock
#
#      G(s) = Ω(s)/V(s) = 3369 / (s + 19.58)      [(grados/s) / voltio]
#
#  Tramas de 50 muestras con Ts = 10 ms (una trama cada 0.5 s).
#
#  ---------------------------------------------------------------------------
#  EL PROBLEMA DE FONDO EN PLUTO
#
#  Una celda de Pluto se ejecuta una vez y muestra su valor de retorno al
#  terminar; no existe `display` incremental. Para animar hay que lograr que la
#  celda se vuelva a ejecutar, y el camino usual es PlutoUI.Clock: un temporizador
#  que dispara la re-ejecución cada N segundos, mire o no si hay datos nuevos.
#
#  Aquí se implementan dos alternativas SIN Clock:
#
#  OPCIÓN A — escucha reactiva con PlutoHooks (recomendada)
#      La tarea que recibe las tramas llama a `set_estado` cada vez que llega
#      una, y eso re-ejecuta la celda. Es un listener de verdad: el redibujo
#      ocurre en el instante en que llega la trama, ni antes ni después, y si la
#      transmisión se detiene la celda simplemente deja de actualizarse.
#      Requiere PlutoHooks + PlutoLinks (Pluto > 0.17.2). El código de la celda
#      está documentado al final de este archivo: los macros de hooks deben
#      expandirse DENTRO de la celda (necesitan su cell id), así que no se pueden
#      envolver en una función de biblioteca.
#
#  OPCIÓN B — reproducción en el navegador con `animacion_html`
#      Los marcos se generan como PNG y se envían todos juntos; un pequeño
#      script los intercambia cada 500 ms dentro de un solo <img>. No re-ejecuta
#      ninguna celda, no necesita paquetes extra ni internos de Pluto, y
#      sobrevive a la exportación a HTML estático. La adquisición deja de ser
#      en vivo (se calcula primero y se reproduce después), pero visualmente es
#      idéntica y es la opción más robusta para una clase.
# =============================================================================

using Plots
using Random
using Printf
using Base64

gr()

# =============================================================================
#  1-3. Modelo y generador de datos sintéticos
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
#  Estado de la adquisición
# -----------------------------------------------------------------------------
mutable struct EstadoMotor
    V::Float64
    T_sim::Float64
    Ts::Float64
    N_trama::Int
    ω_ss::Float64
    t::Vector{Float64}
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
    agregar_trama!(e, t_tr, ω_tr)

Incorpora una trama recibida. Es el punto único de mutación del estado, para
que la celda de Pluto y la tarea de escucha no compitan por los vectores.
"""
function agregar_trama!(e::EstadoMotor, t_tr, ω_tr)
    append!(e.t, t_tr)
    append!(e.ω, ω_tr)
    e.n_tramas += 1
    return e
end

"Vacía sin bloquear las tramas disponibles (útil si se prefiere sondear)."
function actualizar!(e::EstadoMotor)
    while isready(e.canal)
        t_tr, ω_tr = take!(e.canal)
        agregar_trama!(e, t_tr, ω_tr)
    end
    if !isopen(e.canal) && !isready(e.canal)
        e.terminado = true
    end
    return e
end

"Cierra la adquisición antes de tiempo (al re-ejecutar celdas)."
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

# =============================================================================
#  OPCIÓN B — reproducción en el navegador (sin paquetes, sin Clock, sin hooks)
# =============================================================================

"Serializa una figura como PNG en base64 para incrustarla en HTML."
function _png_base64(plt)
    io = IOBuffer()
    show(io, MIME"image/png"(), plt)
    return base64encode(take!(io))
end

"""
    marcos_motor_dc(V=4.0, T_sim=5.0; kwargs...) -> (EstadoMotor, Vector{String})

Genera un marco PNG por trama recibida, consumiendo el canal trama por trama
(iterar el `Channel` bloquea hasta que llega cada una, así que los marcos salen
en el mismo orden en que los transmitiría el procesador).

Se usa `tiempo_real = false` porque aquí no se espera: la temporización la pone
después el navegador.
"""
function marcos_motor_dc(V::Real = 4.0, T_sim::Real = 5.0; kwargs...)
    e = iniciar_motor_dc(V, T_sim; tiempo_real = false, kwargs...)
    marcos = String[_png_base64(graficar(e))]        # marco 0: ejes vacíos

    for (t_tr, ω_tr) in e.canal
        agregar_trama!(e, t_tr, ω_tr)
        push!(marcos, _png_base64(graficar(e)))
    end
    e.terminado = true

    return e, marcos
end

"""
    animacion_html(V=4.0, T_sim=5.0; ms=500, kwargs...) -> HTML

Devuelve un objeto `HTML` que reproduce la adquisición dentro de una sola celda
de Pluto: un `<img>` cuyo `src` cambia cada `ms` milisegundos. Con `ms = 500`
la reproducción va al mismo ritmo que la transmisión real (una trama por cada
0.5 s de datos).

No re-ejecuta celdas, no usa Clock ni hooks, y funciona igual en el notebook
exportado a HTML. Incluye un botón para repetir.
"""
function animacion_html(V::Real = 4.0, T_sim::Real = 5.0; ms::Int = 500, kwargs...)
    e, marcos = marcos_motor_dc(V, T_sim; kwargs...)
    uid  = string("motordc", string(rand(UInt32), base = 16))
    urls = join(("\"data:image/png;base64," * m * "\"" for m in marcos), ",")

    cuerpo = """
    <div style="font-family:sans-serif">
      <img id="$(uid)-img" src="data:image/png;base64,$(first(marcos))"
           style="max-width:100%; display:block">
      <div style="margin-top:6px; display:flex; gap:10px; align-items:center">
        <button id="$(uid)-btn"
                style="padding:4px 12px; cursor:pointer">repetir</button>
        <span id="$(uid)-lbl" style="color:#666; font-size:0.85em"></span>
      </div>
    </div>
    <script>
    (function() {
      const marcos = [$(urls)];
      const img = document.getElementById("$(uid)-img");
      const btn = document.getElementById("$(uid)-btn");
      const lbl = document.getElementById("$(uid)-lbl");
      let cronometro = null;

      function reproducir() {
        if (cronometro !== null) clearInterval(cronometro);
        let k = 0;
        img.src = marcos[0];
        lbl.textContent = "trama 0 de " + (marcos.length - 1);
        cronometro = setInterval(function() {
          k += 1;
          img.src = marcos[k];
          lbl.textContent = "trama " + k + " de " + (marcos.length - 1);
          if (k >= marcos.length - 1) {
            clearInterval(cronometro);
            cronometro = null;
          }
        }, $(ms));
      }

      btn.addEventListener("click", reproducir);
      reproducir();
    })();
    </script>
    """
    return HTML(cuerpo)
end

"""
    resumen(e::EstadoMotor)

Resumen de la adquisición y estimación de la constante de tiempo (63.2 % de ω_ss).
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

# =============================================================================
#  CELDAS DE PLUTO
# -----------------------------------------------------------------------------
#  OPCIÓN B — la más simple, dos celdas:
#
#      # celda 1
#      begin
#          using Plots
#          include("motor_dc_pluto.jl")
#      end
#
#      # celda 2
#      animacion_html(4.0, 5.0)
#
# -----------------------------------------------------------------------------
#  OPCIÓN A — escucha reactiva con PlutoHooks (la celda se actualiza sola):
#
#      # celda 1
#      begin
#          using Plots
#          using PlutoHooks: @use_state, @use_memo
#          using PlutoLinks: @use_task
#          include("motor_dc_pluto.jl")
#      end
#
#      # celda 2  — todo ocurre aquí; re-ejecútela para relanzar la adquisición
#      begin
#          # `version` sólo existe para disparar la re-ejecución de la celda
#          version, set_version = @use_state(0)
#
#          # el estado se crea UNA vez y sobrevive a las re-ejecuciones
#          estado = @use_memo([]) do
#              iniciar_motor_dc(4.0, 5.0)
#          end
#
#          # el listener: iterar el canal bloquea hasta que llega cada trama,
#          # y set_version reactiva la celda en ese preciso instante
#          @use_task([]) do
#              for (t_tr, ω_tr) in estado.canal
#                  agregar_trama!(estado, t_tr, ω_tr)
#                  set_version(estado.n_tramas)
#              end
#              estado.terminado = true
#              set_version(-1)
#          end
#
#          version              # dependencia explícita: sin esto no se redibuja
#          graficar(estado)     # la celda devuelve la figura -> Pluto la muestra
#      end
#
#      # celda 3  (opcional, al terminar)
#      estado.terminado && resumen(estado)
#
#  Notas de la opción A:
#   * `@use_memo([])` y `@use_task([])` con lista de dependencias vacía se
#     ejecutan una sola vez, no en cada re-ejecución de la celda.
#   * Si su versión de PlutoLinks no exporta `@use_task`, use en su lugar
#     `PlutoHooks.@use_effect([]) do ... end` lanzando la tarea con `@async` y
#     devolviendo una función de limpieza que cierre el canal.
#   * Las tareas de Julia son cooperativas, así que `agregar_trama!` y
#     `graficar` no se ejecutan al mismo tiempo mientras no active hilos.
# =============================================================================
