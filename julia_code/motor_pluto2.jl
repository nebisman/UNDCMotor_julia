# =============================================================================
#  Monitor de adquisición en tiempo real para Pluto — sin Clock, sin hooks
#
#  Caso de prueba: motor DC con G(s) = 3369/(s + 19.58)  [(grados/s)/voltio],
#  tramas de 50 muestras con Ts = 10 ms (una trama cada 0.5 s).
#
#  Celdas del notebook (eso es todo):
#
#      # celda 1
#      begin
#          using Plots, HypertextLiteral, AbstractPlutoDingetjes
#          include("motor_dc_pluto.jl")
#      end
#
#      # celda 2
#      monitor_motor_dc(4.0, 5.0)
#
#  ---------------------------------------------------------------------------
#  MECANISMO
#
#  Una celda de Pluto muestra su valor de retorno al terminar; no hay display
#  incremental. En vez de re-ejecutar la celda con un temporizador, aquí la
#  celda devuelve UN widget que se queda vivo y pide datos por su cuenta:
#
#      JS:     src = await pedir()      <- se queda esperando
#      Julia:  take!(canal)             <- bloquea hasta que llega la trama
#                                          y devuelve el PNG de la figura
#
#  Es un long-poll: no hay reloj ni sondeo, el navegador queda a la espera y
#  Julia responde en el instante exacto en que llega el dato. El puente es
#  AbstractPlutoDingetjes.Display.with_js_link. Si no está disponible en su
#  versión, `monitor_motor_dc` cae solo a la reproducción por marcos
#  (`animacion_html`), que no necesita nada especial.
#
#  ---------------------------------------------------------------------------
#  PARA UN SISTEMA CON DATOS REALES
#
#  El widget no sabe nada del motor ni de cómo se dibuja. `monitor_en_vivo`
#  recibe dos cosas y nada más:
#
#      * `figura_inicial`  : qué mostrar antes del primer dato
#      * `siguiente!()`    : función que BLOQUEA hasta el próximo marco y
#                            devuelve la figura, o `nothing` al agotarse la
#                            fuente
#
#  Cambiar el generador sintético por un puerto serial, un socket o un tópico
#  MQTT no toca ni el widget ni `graficar`:
#
#      monitor_en_vivo(graficar(estado)) do
#          trama = leer_puerto_serial()          # bloquea
#          trama === nothing && return nothing   # fin de la transmisión
#          agregar_trama!(estado, trama...)
#          graficar(estado)
#      end
# =============================================================================

using Plots
using Random
using Printf
using Base64
using HypertextLiteral
using AbstractPlutoDingetjes

gr()

# =============================================================================
#  A. Widget genérico de monitoreo
# =============================================================================

"¿Está disponible el puente JS <-> Julia en esta versión de AbstractPlutoDingetjes?"
_hay_enlace_js() = isdefined(AbstractPlutoDingetjes, :Display) &&
                   isdefined(AbstractPlutoDingetjes.Display, :with_js_link)

"Serializa una figura como data URL PNG lista para el atributo src de un <img>."
function _png_data_url(fig)
    io = IOBuffer()
    show(io, MIME"image/png"(), fig)
    return "data:image/png;base64," * base64encode(take!(io))
end

"""
    monitor_en_vivo(siguiente!::Function, figura_inicial; etiqueta = "trama")

Widget de una sola celda que muestra `figura_inicial` y la reemplaza cada vez
que `siguiente!()` entrega un marco nuevo.

`siguiente!()` se llama desde el navegador y debe **bloquear** hasta que haya
datos (con un `Channel`, basta `take!`). Debe devolver la figura, o `nothing`
cuando la fuente se agote; ahí el widget se detiene solo.

El widget nunca sondea ni usa temporizadores: mientras espera no consume nada.
"""
function monitor_en_vivo(siguiente!::Function, figura_inicial; etiqueta::String = "trama")
    _hay_enlace_js() || error("""
        Esta versión de AbstractPlutoDingetjes no expone Display.with_js_link.
        Actualícela (] up AbstractPlutoDingetjes) o use animacion_html(...).""")

    enlace = AbstractPlutoDingetjes.Display.with_js_link() do _
        fig = siguiente!()
        return fig === nothing ? "" : _png_data_url(fig)   # "" = fin de la transmisión
    end

    @htl("""
    <div style="font-family: sans-serif">
      <img src=$(_png_data_url(figura_inicial)) style="max-width: 100%; display: block">
      <span style="color: #666; font-size: 0.85em">esperando datos…</span>
    </div>
    <script>
      const contenedor = currentScript.parentElement
      const img = contenedor.querySelector("img")
      const lbl = contenedor.querySelector("span")
      const pedir = $(enlace)
      const etiqueta = $(etiqueta)   // interpolar SOLO en posición de valor JS

      // si la celda se re-ejecuta o se borra, este bucle debe morir
      let vivo = true
      invalidation.then(() => { vivo = false })

      let k = 0
      try {
        while (vivo) {
          const src = await pedir("siguiente")      // <- espera aquí, sin reloj
          if (!vivo) break
          if (!src) {
            lbl.textContent = "transmisión terminada — " + k + " " + etiqueta + "s"
            break
          }
          k += 1
          img.src = src
          lbl.textContent = etiqueta + " " + k
        }
      } catch (err) {
        lbl.textContent = "error en el enlace: " + err   // fallar en silencio, nunca
        console.error(err)
      }
    </script>
    """)
end

"""
    probar_enlace_js()

Celda de prueba mínima: pide cinco veces la hora a Julia. Si ve el contador
avanzar, `with_js_link` funciona y `monitor_en_vivo` también.
"""
function probar_enlace_js()
    _hay_enlace_js() || return HTML("<b>with_js_link no disponible en esta versión.</b>")

    enlace = AbstractPlutoDingetjes.Display.with_js_link() do _
        sleep(0.5)
        return string("respuesta de Julia a las ", Dates_hora())
    end

    @htl("""
    <div style="font-family: monospace">esperando…</div>
    <script>
      const div = currentScript.parentElement.querySelector("div")
      const pedir = $(enlace)
      for (let i = 1; i <= 5; i++) {
        const txt = await pedir(i)
        div.textContent = i + "/5 — " + txt
      }
    </script>
    """)
end

Dates_hora() = Libc.strftime("%H:%M:%S", time())

# =============================================================================
#  B. 1-3. Fuente de datos: modelo y generador sintético
# =============================================================================
const K_MOTOR = 3369.0    # numerador de G(s)
const P_MOTOR = 19.58     # polo de G(s)  [rad/s]

ganancia_dc()      = K_MOTOR / P_MOTOR   # 172.06 (grados/s)/V
constante_tiempo() = 1 / P_MOTOR         # 51.1 ms

"Cuantización tipo encoder; `q <= 0` deshabilita el efecto."
_cuantizar(x, q) = q > 0 ? q * round(x / q) : x

"""
    tramas_motor_dc(V, T_sim; Ts, N_trama, σ_ruido, resolucion, tiempo_real, semilla, buffer)

Emula el procesador que muestrea el sensor y transmite en tramas de `N_trama`
muestras. Devuelve un `Channel` que entrega tuplas `(t, ω)`.

Discretización exacta (ZOH) de `G(s)`, válida en los instantes de muestreo:

    ω[k+1] = a·ω[k] + b·V ,   a = e^(-p·Ts),   b = (K/p)·(1 - a)

En el sistema real, esta función es la única que se reemplaza.
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
            put!(canal, (t_tr, ω_tr))
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

"Incorpora una trama recibida. Punto único de mutación del estado."
function agregar_trama!(e::EstadoMotor, t_tr, ω_tr)
    append!(e.t, t_tr)
    append!(e.ω, ω_tr)
    e.n_tramas += 1
    return e
end

"Cierra la adquisición antes de tiempo."
function detener!(e::EstadoMotor)
    isopen(e.canal) && close(e.canal)
    e.terminado = true
    return e
end

# -----------------------------------------------------------------------------
#  4, 7, 8. Graficación — encapsulada: la celda del notebook nunca la menciona
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
#  Puente entre la fuente y el widget
# -----------------------------------------------------------------------------
"""
    siguiente_figura!(e, graficador) -> figura | nothing

Bloquea en `take!` hasta que llega la próxima trama y devuelve la figura
actualizada. Cuando el productor cierra el canal devuelve `nothing`, que es la
señal de fin que espera `monitor_en_vivo`.
"""
function siguiente_figura!(e::EstadoMotor, graficador::Function = graficar)
    try
        t_tr, ω_tr = take!(e.canal)        # <- aquí se espera, sin reloj ni sondeo
        agregar_trama!(e, t_tr, ω_tr)
        return graficador(e)
    catch err
        err isa InvalidStateException || rethrow()
        e.terminado = true                 # canal cerrado: no hay más datos
        return nothing
    end
end

# =============================================================================
#  C. Punto de entrada del notebook
# =============================================================================
"""
    monitor_motor_dc(V=4.0, T_sim=5.0; graficador=graficar, kwargs...)

Única llamada que necesita la celda de Pluto. Arranca la adquisición y devuelve
el widget que se actualiza al llegar cada trama.

Si `with_js_link` no está disponible cae automáticamente a `animacion_html`.
El estado queda accesible después en `estado_actual()` para el análisis.
"""
function monitor_motor_dc(V::Real = 4.0, T_sim::Real = 5.0;
                          graficador::Function = graficar, kwargs...)
    if !_hay_enlace_js()
        @warn "with_js_link no disponible: se usa la reproducción por marcos."
        return animacion_html(V, T_sim; kwargs...)
    end

    e = iniciar_motor_dc(V, T_sim; kwargs...)
    _ULTIMO_ESTADO[] = e

    return monitor_en_vivo(() -> siguiente_figura!(e, graficador), graficador(e))
end

const _ULTIMO_ESTADO = Ref{Union{Nothing,EstadoMotor}}(nothing)

"Estado de la última adquisición lanzada, para inspeccionar los datos al terminar."
estado_actual() = _ULTIMO_ESTADO[]

"""
    probar_fuente(V=4.0, T_sim=5.0; n=3)

Consume `n` tramas desde Julia, sin navegador. Sirve para separar los dos lados
del problema: si esto imprime tramas, el productor y `siguiente_figura!` están
bien y cualquier falla restante está en el widget.
"""
function probar_fuente(V::Real = 4.0, T_sim::Real = 5.0; n::Int = 3, kwargs...)
    e = iniciar_motor_dc(V, T_sim; kwargs...)
    for i in 1:n
        fig = siguiente_figura!(e)
        if fig === nothing
            println("la fuente se agotó en el intento $i")
            break
        end
        @printf("trama %d: t = %.2f s, ω = %6.1f grados/s\n",
                e.n_tramas, last(e.t), last(e.ω))
    end
    detener!(e)
    return e
end

# =============================================================================
#  D. Respaldo: reproducción por marcos (sin with_js_link, sin Clock)
# =============================================================================
"""
    animacion_html(V=4.0, T_sim=5.0; ms=500, graficador=graficar, kwargs...)

Genera todos los marcos y los reproduce en el navegador cada `ms` ms dentro de
un solo `<img>`. No es adquisición en vivo —se calcula primero y se reproduce
después— pero no depende de ninguna API de Pluto y sobrevive a la exportación
a HTML estático.
"""
function animacion_html(V::Real = 4.0, T_sim::Real = 5.0;
                        ms::Int = 500, graficador::Function = graficar, kwargs...)
    e = iniciar_motor_dc(V, T_sim; tiempo_real = false, kwargs...)
    _ULTIMO_ESTADO[] = e

    marcos = String[_png_data_url(graficador(e))]
    for (t_tr, ω_tr) in e.canal
        agregar_trama!(e, t_tr, ω_tr)
        push!(marcos, _png_data_url(graficador(e)))
    end
    e.terminado = true

    return @htl("""
    <div style="font-family: sans-serif">
      <img src=$(first(marcos)) style="max-width: 100%; display: block">
      <div style="margin-top: 6px; display: flex; gap: 10px; align-items: center">
        <button style="padding: 4px 12px; cursor: pointer">repetir</button>
        <span style="color: #666; font-size: 0.85em"></span>
      </div>
    </div>
    <script>
      const cont = currentScript.parentElement
      const img  = cont.querySelector("img")
      const btn  = cont.querySelector("button")
      const lbl  = cont.querySelector("span")
      const marcos = $(marcos)
      let crono = null

      function reproducir() {
        if (crono !== null) clearInterval(crono)
        let k = 0
        img.src = marcos[0]
        lbl.textContent = "trama 0 de " + (marcos.length - 1)
        crono = setInterval(() => {
          k += 1
          img.src = marcos[k]
          lbl.textContent = "trama " + k + " de " + (marcos.length - 1)
          if (k >= marcos.length - 1) { clearInterval(crono); crono = null }
        }, $(ms))
      }

      invalidation.then(() => { if (crono !== null) clearInterval(crono) })
      btn.addEventListener("click", reproducir)
      reproducir()
    </script>
    """)
end

"""
    resumen(e = estado_actual())

Resumen de la adquisición y estimación de la constante de tiempo (63.2 % de ω_ss).
"""
function resumen(e::EstadoMotor = estado_actual())
    @printf("Muestras: %d   |   tramas: %d   |   ω final: %.1f grados/s   |   ω_ss teórica: %.1f grados/s\n",
            length(e.t), e.n_tramas, last(e.ω), e.ω_ss)
    k63 = findfirst(≥(0.632 * e.ω_ss), e.ω)
    k63 === nothing && return nothing
    @printf("τ teórica: %.1f ms   |   τ estimada de los datos: %.1f ms\n",
            constante_tiempo() * 1e3, e.t[k63] * 1e3)
    return nothing
end