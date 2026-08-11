# UNDCMotor.jl

Paquete de Julia para control e identificación del sistema **UNDCMotor** — un motor DC controlado por un ESP32 — vía comunicación serial (USB).

Equivalente funcional de la librería Python (`motorsys.py`, `controlsys.py`, `identsys.py`).

---

## Requisitos previos

| Requisito | Versión mínima | Notas |
|-----------|---------------|-------|
| **Julia** | 1.9+ | Descargar de [julialang.org](https://julialang.org/downloads/) |
| **ESP32** | — | Con firmware `esp32_unmotor.ino` cargado |
| **Cable USB** | — | Conectado al ESP32 |
| **Permisos serial** (Linux) | — | El usuario debe pertenecer al grupo `dialout` |

### Permisos del puerto serial en Linux

```bash
sudo usermod -a -G dialout $USER
# Reiniciar sesión o hacer:
newgrp dialout
```

---

## Instalación

### Opción A: Script automático

```bash
cd UNDCMotor
julia setup.jl
```

### Opción B: Manual desde el REPL de Julia

```julia
# 1. Instalar dependencias
import Pkg
Pkg.add([
    "LibSerialPort",
    "JSON3",
    "Interpolations",
    "DelimitedFiles",
    "ControlSystems",
    "MatrixEquations",
    "Optim",
    "Plots",
])

# 2. Activar el paquete
cd("ruta/a/UNDCMotor")
Pkg.activate(".")
Pkg.instantiate()
```

### Opción C: Desde Jupyter con IJulia

```julia
import Pkg
Pkg.activate("ruta/a/UNDCMotor")
Pkg.instantiate()
using UNDCMotor
```

---

## Paquetes de Julia utilizados

| Paquete | Equivalente Python | Propósito |
|---------|--------------------|-----------|
| `LibSerialPort` | `pyserial` | Comunicación serial con ESP32 |
| `JSON3` | `json` | Parsing de tramas JSON |
| `ControlSystems` | `python-control` | Funciones de transferencia, espacio de estados, `lsim`, `c2d` |
| `MatrixEquations` | `scipy.linalg (dare)` | Ecuación de Riccati para anti-windup |
| `Optim` | `scipy.optimize` | Optimización para identificación |
| `Interpolations` | `scipy.interpolate` | Interpolación de curva estática |
| `Plots` | `matplotlib` | Gráficas en tiempo real |
| `DelimitedFiles` | `csv` | Lectura/escritura CSV |
| `LinearAlgebra` | `numpy.linalg` | Operaciones matriciales |
| `Statistics` | `numpy (mean, std)` | Estadísticas básicas |

---

## Estructura del paquete

```
UNDCMotor/
├── Project.toml            # Manifiesto de dependencias
├── setup.jl                # Script de instalación
├── README.md               # Este archivo
├── src/
│   ├── UNDCMotor.jl        # Módulo principal
│   ├── motorsys.jl         # Comunicación serial + conversiones hex
│   ├── controlsys.jl       # Funciones de control
│   └── identsys.jl         # Funciones de identificación
├── examples/
│   ├── ejemplo_basico.jl           # PID, step, escaleras, perfil
│   ├── ejemplo_identificacion.jl   # Curva estática, FOTD, PRBS
│   └── ejemplo_controlador.jl      # Diseño y carga de controladores
├── datafiles/              # Datos de la curva estática (generados)
└── experiment_files/       # Archivos CSV de experimentos (generados)
```

---

## Uso rápido

### 1. Crear el sistema y configurar PID

```julia
using UNDCMotor

# Crear la interfaz (ajustar puerto según SO)
sys = MotorSystemIoT(
    plant_number = "LEO",
    port         = "/dev/ttyUSB0",   # Linux
    # port       = "COM3",           # Windows
    # port       = "/dev/cu.usbserial-XXX",  # macOS
    bauds        = 921600,
)

# Configurar PID para control de posición
set_pid!(sys;
    kp = 0.146, ki = 0.732, kd = 0.014,
    N = 10.0, beta = 0.7,
    output = :angle,        # :angle o :speed
    deadzone = 0.125,
)

# Fijar referencia
set_reference!(sys, 90.0)
```

### 2. Escalón en lazo cerrado

```julia
t, r, y, u = step_closed(sys; r0=0, r1=180, t0=0.5, t1=2.0)
```

La gráfica se actualiza en **tiempo real** mientras llegan datos del ESP32.

### 3. Escaleras en lazo cerrado

```julia
t, r, y, u = stairs_closed(sys;
    stairs   = (0, 90, 180, 270, 180, 90, 0),
    duration = 1.5,
)
```

### 4. Perfil arbitrario

```julia
t, r, y, u = profile_closed(sys;
    timevalues = (0, 1, 2, 3, 4, 5),
    refvalues  = (0, 360, 360, 0, -180, 0),
)
```

### 5. Identificación — Escalón en lazo abierto

```julia
t, u, y = step_open(sys; u0=1.5, u1=3.5, t0=1.0, t1=1.0)
```

### 6. Identificación — PRBS

```julia
t, u, y = prbs_open(sys; low_val=2.0, high_val=4.0, divider=2)
```

### 7. Modelo estático (curva velocidad vs voltaje)

```julia
uee, yee = get_static_model(sys; points=30)
```

### 8. Modelo FOTD desde escalón

```julia
alpha, tau, L = get_fomodel_step(sys; yop=400)
# G(s) = alpha / (tau·s + 1) · exp(-L·s)
```

### 9. Modelo de primer orden desde PRBS

```julia
G1 = get_models_prbs(sys; yop=400)
```

### 10. Diseñar y cargar un controlador general

```julia
using ControlSystems

s = tf("s")
C = 0.005 + 0.05/s    # Controlador PI

# Cargar al ESP32 (1 DOF, control de velocidad)
set_controller!(sys, C; output=:speed, deadzone=0.0)

# Verificar con un escalón
t, r, y, u = step_closed(sys; r0=0, r1=400, t0=0.0, t1=3.0)
```

### 11. Función de transferencia nominal (sin hardware)

```julia
Gpos = transfer_function(sys; output=:position)
Gvel = transfer_function(sys; output=:speed)
bodeplot(Gvel)
```

---

## Correspondencia Python → Julia

| Python | Julia | Notas |
|--------|-------|-------|
| `system = MotorSystemIoT(...)` | `sys = MotorSystemIoT(...)` | Keyword arguments |
| `set_pid(system, kp=1, ...)` | `set_pid!(sys; kp=1, ...)` | `!` indica mutación |
| `set_reference(system, 90)` | `set_reference!(sys, 90)` | |
| `set_controller(system, C, output='speed')` | `set_controller!(sys, C; output=:speed)` | Símbolo `:speed` |
| `step_closed(system, r0=0, r1=100)` | `step_closed(sys; r0=0, r1=100)` | |
| `stairs_closed(system, stairs=[90,180])` | `stairs_closed(sys; stairs=(90,180))` | Tupla o vector |
| `step_open(system, u0=1, u1=3)` | `step_open(sys; u0=1, u1=3)` | |
| `prbs_open(system, low_val=2)` | `prbs_open(sys; low_val=2)` | |
| `get_static_model(system)` | `get_static_model(sys)` | |
| `get_fomodel_step(system, yop=400)` | `get_fomodel_step(sys; yop=400)` | |
| `get_models_prbs(system, yop=400)` | `get_models_prbs(sys; yop=400)` | |
| `ct.tf(num, den)` | `tf(num, den)` | ControlSystems.jl |
| `ct.forced_response(G, t, u)` | `lsim(G, u', t)` | Entrada como fila |

---

## Protocolo serial

La comunicación con el ESP32 sigue el protocolo de dos líneas (igual que el firmware):

```
<topic>\n
<json_payload>\n
```

Por ejemplo, para configurar el PID:
```
LEO/set_pid
{"kp":"3e800000","ki":"3f000000",...}
```

Los valores float se transmiten como hexadecimal big-endian IEEE-754 de 32 bits
para evitar pérdida de precisión.

Las respuestas del ESP32 son líneas JSON con campos `u`, `y`, `r` (hex separados por comas)
y `frame` (número de trama en hex).

---

## Solución de problemas

**"No se recibieron datos del ESP32"**
- Verificar que el cable USB esté conectado
- Verificar que el puerto sea correcto (`ls /dev/tty*` en Linux)
- Verificar que el firmware esté cargado y funcionando
- Verificar que el baudrate coincida (921600 por defecto)

**"Permission denied" al abrir el puerto**
- Linux: `sudo usermod -a -G dialout $USER` y reiniciar sesión
- macOS: normalmente no requiere permisos adicionales
- Windows: instalar el driver USB del ESP32

**Las gráficas no se actualizan en tiempo real**
- Usar el backend `gr()` de Plots.jl (por defecto)
- En Jupyter, usar `plotlyjs()` o `gr()` con `%matplotlib widget`

---

## Licencia

MIT License — LB 2026
