# ═══════════════════════════════════════════════════════════════════════════════
#  motorsys.jl – Comunicación serial con el sistema UNDCMotor (ESP32).
#  LB 2026 – MIT License
# ═══════════════════════════════════════════════════════════════════════════════

using LibSerialPort
using JSON3
using Interpolations
using DelimitedFiles
using Printf
using Statistics

# ── Constantes del protocolo ─────────────────────────────────────────────────
const SAMPLING_TIME = 0.02
const BUFFER_SIZE   = 20
const PRBS_LENGTH   = 1023
const FONT_SIZE     = 12

# ── Rutas por defecto ────────────────────────────────────────────────────────
const PATH_DEFAULT = joinpath(@__DIR__, "..", "experiment_files") * "/"
const PATH_DATA    = joinpath(@__DIR__, "..", "datafiles") * "/"

function _ensure_dirs()
    mkpath(PATH_DEFAULT)
    mkpath(PATH_DATA)
end


# ═══════════════════════════════════════════════════════════════════════════════
#  Funciones de conversión hex  (idénticas al protocolo del firmware)
#  El firmware usa big-endian IEEE-754 float32 y big-endian uint32.
#  Nota: reinterpret(UInt32, Float32(x)) devuelve el bit-pattern IEEE-754,
#  que coincide con la representación big-endian usada por Python struct.pack('>f').
# ═══════════════════════════════════════════════════════════════════════════════

"""Convierte Float32 a string hexadecimal de 8 caracteres (bit-pattern IEEE-754)."""
function float2hex(value::Real)::String
    u = reinterpret(UInt32, Float32(value))
    return string(u; base=16, pad=8)
end

"""Convierte string hexadecimal de 8 caracteres a Float64 (bit-pattern IEEE-754)."""
function hex2float(h::AbstractString)::Float64
    u = parse(UInt32, h; base=16)
    return Float64(reinterpret(Float32, u))
end

"""Convierte UInt32 a string hexadecimal de 8 caracteres."""
function long2hex(value::Integer)::String
    return string(UInt32(value); base=16, pad=8)
end

"""Convierte string hexadecimal de 8 caracteres a Int."""
function hex2long(h::AbstractString)::Int
    return Int(parse(UInt32, h; base=16))
end

"""Convierte un vector de floats a un string hex concatenado."""
function signal2hex(signal)::String
    return join(float2hex(v) for v in signal)
end

"""Convierte un vector de enteros (puntos de tiempo) a string hex concatenado."""
function time2hex(time_points)::String
    return join(long2hex(Int(t)) for t in time_points)
end

"""Convierte una matriz (2D array) a string hex fila por fila."""
function matrix2hex(mat)::String
    if ndims(mat) == 1
        return join(float2hex(e) for e in mat)
    else
        return join(float2hex(mat[i, j]) for i in axes(mat, 1) for j in axes(mat, 2))
    end
end

"""Convierte un string hex separado por comas a un vector de Float64."""
function hexframe_to_array(hexframe::AbstractString)::Vector{Float64}
    return [hex2float(strip(p)) for p in split(hexframe, ",")]
end


# ═══════════════════════════════════════════════════════════════════════════════
#  Lectura de archivos CSV
# ═══════════════════════════════════════════════════════════════════════════════

"""Lee archivo CSV de 2 columnas (u, y). Retorna (u, y)."""
function read_csv_file(filepath::String = PATH_DATA * "DCmotor_static_gain_response.csv")
    data = readdlm(filepath, ','; header=true)[1]
    return Float64.(data[:, 1]), Float64.(data[:, 2])
end

"""Lee archivo CSV de 3 columnas (t, u, y). Retorna (t, u, y)."""
function read_csv_file3(filepath::String)
    data = readdlm(filepath, ','; header=true)[1]
    return Float64.(data[:, 1]), Float64.(data[:, 2]), Float64.(data[:, 3])
end


# ═══════════════════════════════════════════════════════════════════════════════
#  Estructura principal:  MotorSystem
# ═══════════════════════════════════════════════════════════════════════════════

"""
    MotorSystem(;  port, bauds)

Interfaz serial con el sistema UNDCMotor (ESP32).
Equivalente a la clase MotorSystem de Python.
"""
mutable struct MotorSystem   
    port         :: String
    bauds        :: Int
    sp           :: Union{SerialPort, Nothing}
    topics       :: Dict{String, String}
    queue        :: Channel{Dict{String, Any}}
    reader_task  :: Union{Task, Nothing}
    stop_reader  :: Threads.Atomic{Bool}

    function MotorSystem(;                        
            port::String         = "/dev/ttyUSB0",
            bauds::Int           = 460800)

        _ensure_dirs()
        
        topics = Dict(
            "set_pid"       => "/set_pid",
            "set_ref"       => "/set_ref",
            "step_closed"   => "/step_closed",
            "stairs_closed" => "/stairs_closed",
            "prbs_open"     => "/prbs_open",
            "step_open"     => "/step_open",
            "set_gencon"    => "/set_gencon",
            "prof_closed"   => "/prof_closed",
        )
        new(port, bauds, nothing, topics,
            Channel{Dict{String, Any}}(256), nothing, Threads.Atomic{Bool}(false))
    end
end

# ── Conexión / desconexión ───────────────────────────────────────────────────

"""Abre el puerto serial e inicia la tarea lectora."""
function connect!(sys::MotorSystem)
    if sys.sp === nothing
        sys.sp = LibSerialPort.open(sys.port, sys.bauds)
        # Limpiar buffer de entrada (ignorar si falla)
        try sp_flush(sys.sp, SP_BUF_INPUT) catch end
    end
    _start_reader!(sys)
    return nothing
end

"""Detiene la tarea lectora y cierra el puerto serial."""
function disconnect!(sys::MotorSystem)
    _stop_reader!(sys)
    if sys.sp !== nothing
        close(sys.sp)
        sys.sp = nothing
    end
    return nothing
end

# ── Hilo lector en segundo plano ─────────────────────────────────────────────

function _start_reader!(sys::MotorSystem)
    if sys.reader_task !== nothing && !istaskdone(sys.reader_task)
        return  # ya corriendo
    end
    sys.stop_reader[] = false
    # Vaciar la cola de datos viejos
    while isready(sys.queue)
        take!(sys.queue)
    end
    sys.reader_task = Threads.@spawn _reader_loop(sys)
    return nothing
end

function _stop_reader!(sys::MotorSystem)
    sys.stop_reader[] = true
    if sys.reader_task !== nothing
        # Esperar hasta 3 segundos
        t0 = time()
        while !istaskdone(sys.reader_task) && (time() - t0) < 3.0
            sleep(0.05)
        end
        sys.reader_task = nothing
    end
    return nothing
end

function _reader_loop(sys::MotorSystem)
    buf = IOBuffer()
    while !sys.stop_reader[]
        try
            # Leer bytes disponibles del serial
            if sys.sp === nothing
                break
            end
            nb = bytesavailable(sys.sp)
            if nb == 0
                sleep(0.001)
                continue
            end
            bytes = read(sys.sp, nb)
            write(buf, bytes)

            # Procesar líneas completas
            raw = String(take!(buf))
            lines = split(raw, '\n')
            # La última parte puede ser incompleta → devolverla al buffer
            if !endswith(raw, '\n')
                write(buf, lines[end])
                lines = lines[1:end-1]
            end

            for line in lines
                line = strip(line)
                isempty(line) && continue

                # Extraer la parte JSON
                json_str = if startswith(line, "D:")
                    line[3:end]
                elseif startswith(line, "{")
                    line
                else
                    continue  # línea de debug → ignorar
                end
               
                msg = try
                    JSON3.read(json_str, Dict{String, Any})
                catch
                    continue
                end

                # Solo tramas con campo "frame"
                if haskey(msg, "frame")
                    try
                        put!(sys.queue, msg)
                    catch
                        break
                    end
                end
            end
        catch e
            if e isa InterruptException
                break
            end
            # Otros errores → continuar
            sleep(0.01)
        end
    end
end

# ── Envío de comandos ────────────────────────────────────────────────────────

"""
    send_command!(sys, topic_key, payload)

Envía `topic\\npayload_json\\n` al firmware (protocolo serialCommandTask).
"""
function send_command!(sys::MotorSystem, topic_key::String, payload::Dict)
    topic = sys.topics[topic_key]
    raw = topic * "\n" * JSON3.write(payload) * "\n"
    write(sys.sp, raw)
    return nothing
end

# ── Recepción síncrona de tramas ─────────────────────────────────────────────

"""
    receive_frames!(sys, total_frames, on_frame; timeout_factor=20.0)

Bloquea hasta recibir `total_frames` tramas. Invoca `on_frame(msg, frame_no)`
por cada trama recibida. Lanza `TimeoutError` si no llegan datos.
"""
function receive_frames!(sys::MotorSystem, total_frames::Int, on_frame::Function;
                         timeout_factor::Float64 = 20.0)
    timeout = timeout_factor * BUFFER_SIZE * SAMPLING_TIME
    curr_frame = -1
    sync = false

    while curr_frame < total_frames 
        msg = try
            # Esperar con timeout usando timedwait
            result = nothing
            deadline = time() + timeout
            while time() < deadline
                if isready(sys.queue)
                    result = take!(sys.queue)
                    break
                end
                sleep(0.005)
            end
            if result === nothing
                error("Timeout")
            end
            result
        catch
            throw(ErrorException(
                "No se recibieron datos del ESP32. " *
                "Verifique la conexión y que el firmware envíe las tramas por Serial."))
        end

        curr_frame = hex2long(string(msg["frame"]))
        if curr_frame == 1
            sync = true
        end
        if sync
            on_frame(msg, curr_frame)
        end
    end
    return nothing
end

# ── Modelos de la planta ─────────────────────────────────────────────────────

"""
    transfer_function(sys; output=:position, min_order=true)

Retorna la función de transferencia nominal del motor DC.
`output` puede ser `:position` o `:speed`.
"""
function transfer_function(sys::MotorSystem,
                           output::Symbol = :angle)
    if output == :angle   # angle   
            num = [3479.0413]
            den = [1.0000, 2.4825, 0]
 
            

    else # :speed at 360
            num = [2300.5004]
            den = [1.0, 3.3203]
       
    end
    return ControlSystems.tf(num, den)
end

"""
    speed_from_volts(sys, volts)

Interpola la velocidad estacionaria a partir de la curva estática.
"""
function speed_from_volts(sys::MotorSystem, volts::Real)
    u, y = read_csv_file()
    itp = linear_interpolation(u, y; extrapolation_bc=Line())
    return itp(Float64(volts))
end

"""
    volts_from_speed(sys, speed)

Interpola el voltaje necesario para una velocidad estacionaria dada.
"""
function volts_from_speed(sys::MotorSystem, speed::Real)
    u, y = read_csv_file()
    speed == 0 && return 0.0
    # Interpolación inversa: u(y)
    # Necesitamos que y sea monótona → separar ramas si es necesario
    # Para este motor, y es monótonamente creciente con u
    itp = linear_interpolation(y, u; extrapolation_bc=Line())
    if y[1] <= speed <= y[end]
        return itp(Float64(speed))
    end
    error("speed debe estar en [$(round(y[1],digits=1)), $(round(y[end],digits=1))]")
end


# ── Guardar experimentos ─────────────────────────────────────────────────────

"""Guarda las columnas como CSV en PATH_DATA y PATH_DEFAULT."""
function save_experiment(columns::Vector, filename::String, header::String)
    mat = hcat(columns...)
    for path in (PATH_DATA, PATH_DEFAULT)
        open(path * filename, "w") do io
            println(io, header)
            for i in axes(mat, 1)
                println(io, join([@sprintf("%.8f", mat[i, j]) for j in axes(mat, 2)], ","))
            end
        end
    end
    return nothing
end
