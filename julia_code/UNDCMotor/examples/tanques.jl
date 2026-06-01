import Pkg
Pkg.activate(joinpath(@__DIR__, ".."))
using UNDCMotor
include("ControlUN.jl")



# Modelo del sistema de tanques en lazo abierto
Aest = [-5.0  0.0 0.0;
        1.0 0.0 0.0;
        0.0 -1.0 0.0]
Best = [5000.0; 0.0; 0.0]


polos_lc =  [-5.0 -6.0 -7.0]
K = place(Aest, Best, polos_lc)


tanque_la = ss(A, B, C, D)

println("Sistema de tanques en lazo abierto:\n")
print(tanque_la)

println("\nValores propios del sistema de tanques:")
print((eigvals(A)))



# Asignación de polos para el sistema de tanques
polos_lc =  [-0.7+0.7im, -.7 -0.7im]

K = place(A, B, polos_lc)


println("Ganancia K para tanques:")
print(K)

# Verificar
println("\nValores propios en lazo cerrado:")
print(eigvals(A - B * K))

# Ganancia de prealimentación
ref = 0.5
kr = 1/(-C * inv(A - B*K) * B)[1]
println("\nGanancia de prealimentación kr = ", round(kr, digits=4))


# Sistema de tanques en lazo cerrado
tanques_lc = ss(A - B *K, B * kr, C, D)

# Respuesta al escalón de amplitud r
t = 0:0.1:30.0
Y, t, X = step(tanques_lc*ref , t)


plot(t, Y',
     xlabel="Tiempo [s]", ylabel="Nivel y₂",
     title="Respuesta al escalón — Sistema de Tanques en Lazo Cerrado",
     label="Salida y", lw=2, color=:blue)
hline!([ref], label="Referencia r=$(ref)", lw=1, ls=:dash, color=:green)


u_control = kr * ref .- (K * X)   # (1 × nt)

u_max = maximum(abs.(u_control))
println("Máximo valor absoluto de la señal de control: ", round(u_max, digits=4))

p1 = plot(t, Y',
          xlabel="Tiempo [s]", ylabel="Nivel y₂",
          title="Salida del sistema",
          label="y", lw=2, color=:blue)
hline!(p1, [ref], label="Referencia", lw=1, ls=:dash, color=:gray)

p2 = plot(t, u_control',
          xlabel="Tiempo [s]", ylabel="u",
          title="Señal de control u = kr*r - K*x",
          label="u(t)", lw=2, color=:red)

plot(p1, p2, layout=(2,1), size=(700, 500))