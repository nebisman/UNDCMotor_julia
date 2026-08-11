#=
Aproximación polinomial de segundo orden para el tiempo de subida (10%→90%)
de un sistema canónico de segundo orden subamortiguado.

    H(s) = ωn² / (s² + 2ζωn·s + ωn²)

El tiempo de subida normalizado  tr·ωn  depende solo de ζ.
Se ajusta:  tr·ωn ≈ a₀ + a₁·ζ + a₂·ζ²

Paquetes necesarios:
    using Pkg
    Pkg.add("Plots")
=#

using Printf
using LinearAlgebra
using Plots; gr()   # backend GR para guardar PNG

# ──────────────────────────────────────────────────────
# 1. Respuesta al escalón analítica (sistema subamortiguado)
# ──────────────────────────────────────────────────────
"""
    step_response(ζ, ωn, t)

Respuesta al escalón unitario del sistema canónico de 2° orden.
Válida para 0 < ζ < 1 (subamortiguado).
"""
function step_response(ζ::Real, ωn::Real, t::AbstractVector)
    σ  = ζ * ωn
    ωd = ωn * sqrt(1 - ζ^2)
    ϕ  = acos(ζ)
    y  = @. 1.0 - exp(-σ * t) / sqrt(1 - ζ^2) * sin(ωd * t + ϕ)
    return y
end

# ──────────────────────────────────────────────────────
# 2. Cálculo exacto del tiempo de subida (10% → 90%)
# ──────────────────────────────────────────────────────
"""
    rise_time_exact(ζ; ωn=1.0, t_final=50.0, N=100_000)

Calcula tr (10%→90%) con interpolación lineal sobre la respuesta analítica.
"""
function rise_time_exact(ζ::Real; ωn::Real=1.0, t_final::Real=50.0, N::Int=100_000)
    t = range(0.0, t_final; length=N)
    y = step_response(ζ, ωn, collect(t))

    # --- Cruce del 10% ---
    i10 = findfirst(y .≥ 0.1)
    isnothing(i10) && return NaN
    if i10 > 1
        t10 = t[i10-1] + (0.1 - y[i10-1]) / (y[i10] - y[i10-1]) * (t[i10] - t[i10-1])
    else
        t10 = t[i10]
    end

    # --- Cruce del 90% ---
    i90 = findfirst(y .≥ 0.9)
    isnothing(i90) && return NaN
    if i90 > 1
        t90 = t[i90-1] + (0.9 - y[i90-1]) / (y[i90] - y[i90-1]) * (t[i90] - t[i90-1])
    else
        t90 = t[i90]
    end

    return t90 - t10
end

# ──────────────────────────────────────────────────────
# 3. Ajuste polinomial por mínimos cuadrados ponderados
# ──────────────────────────────────────────────────────
"""
    polyfit_weighted(x, y, deg, w)

Ajuste polinomial ponderado de grado `deg`.
Devuelve coeficientes [a₀, a₁, …, a_deg] (potencias crecientes).
"""
function polyfit_weighted(x::AbstractVector, y::AbstractVector, deg::Int,
                          w::AbstractVector=ones(length(x)))
    n = length(x)
    # Matriz de Vandermonde [1  x  x²  …  x^deg]
    A = hcat([x .^ k for k in 0:deg]...)
    # Matriz diagonal de pesos
    W = Diagonal(w)
    # Mínimos cuadrados ponderados: (AᵀWA)⁻¹ AᵀWy
    coeffs = (A' * W * A) \ (A' * W * y)
    return coeffs   # [a₀, a₁, a₂]
end

"""
    polyeval(coeffs, x)

Evalúa polinomio con coeficientes [a₀, a₁, a₂, …] en x.
"""
polyeval(c, x) = sum(c[k+1] .* x .^ k for k in 0:length(c)-1)

# ──────────────────────────────────────────────────────
# 4. Generar datos: ζ vs tr·ωn  (con ωn=1 → tr·ωn = tr)
# ──────────────────────────────────────────────────────
ζ_fit  = collect(range(0.05, 0.99; length=200))
tr_exact_vec = [rise_time_exact(z; ωn=1.0) for z in ζ_fit]
tr_norm = tr_exact_vec   # ωn = 1 ⟹ tr·ωn = tr

# Filtrar NaN
valid = .!isnan.(tr_norm)
ζ_v   = ζ_fit[valid]
tr_v  = tr_norm[valid]

# ──────────────────────────────────────────────────────
# 5. Ajuste SIN pesos (referencia)
# ──────────────────────────────────────────────────────
c_nw = polyfit_weighted(ζ_v, tr_v, 2)                # [a₀, a₁, a₂]
poly_nw = polyeval(c_nw, ζ_fit)

# ──────────────────────────────────────────────────────
# 6. Ajuste CON pesos: priorizar ζ ∈ [0.5, 1]
# ──────────────────────────────────────────────────────
const WEIGHT_LOW  = 1.0
const WEIGHT_HIGH = 10.0

valz = 0.5

weights = [z ≥ valz ? WEIGHT_HIGH : WEIGHT_LOW for z in ζ_v]
c_w = polyfit_weighted(ζ_v, tr_v, 2, weights)        # [a₀, a₁, a₂]
poly_w = polyeval(c_w, ζ_fit)

# ──────────────────────────────────────────────────────
# 7. Imprimir resultados
# ──────────────────────────────────────────────────────
println("=" ^ 60)
println("APROXIMACIÓN POLINOMIAL DE 2do ORDEN PARA EL TIEMPO DE SUBIDA")
println("  CON PONDERACIÓN ALTA EN ζ ∈ [0.5, 1]")
println("=" ^ 60)
@printf("\n  Pesos: ζ<0.5 → %.1f,  ζ≥0.5 → %.1f\n", WEIGHT_LOW, WEIGHT_HIGH)
@printf("\n  tr·ωn ≈ %+.6f  %+.6f·ζ  %+.6f·ζ²\n\n", c_w[1], c_w[2], c_w[3])
println("  Coeficientes (ponderado):")
@printf("    a₀ = %.6f\n", c_w[1])
@printf("    a₁ = %.6f\n", c_w[2])
@printf("    a₂ = %.6f\n", c_w[3])

# ──────────────────────────────────────────────────────
# 8. Verificación: error del ajuste por regiones
# ──────────────────────────────────────────────────────
err_w   = tr_v .- polyeval(c_w, ζ_v)
err_nw  = tr_v .- polyeval(c_nw, ζ_v)
epct_w  = 100.0 .* err_w  ./ tr_v
epct_nw = 100.0 .* err_nw ./ tr_v

lo = ζ_v .< valz
hi = ζ_v .≥ valz

# Función auxiliar (evita dependencia de Statistics.jl)
_mean(x) = sum(x) / length(x)

println()
@printf("  %-34s %12s %12s\n", "", "Sin peso", "Con peso")
println("  " * "-" ^ 58)
@printf("  %-34s %10.2f%%  %10.2f%%\n", "Error %% máx GLOBAL",
        maximum(abs.(epct_nw)), maximum(abs.(epct_w)))
@printf("  %-34s %10.2f%%  %10.2f%%\n", "Error %% medio GLOBAL",
        _mean(abs.(epct_nw)), _mean(abs.(epct_w)))
@printf("  %-34s %11.6f  %11.6f\n", "RMSE GLOBAL",
        sqrt(_mean(err_nw .^ 2)), sqrt(_mean(err_w .^ 2)))
println()
@printf("  %-34s %10.2f%%  %10.2f%%\n", "Error %% máx  [ζ < 0.5]",
        maximum(abs.(epct_nw[lo])), maximum(abs.(epct_w[lo])))
@printf("  %-34s %10.2f%%  %10.2f%%\n", "Error %% medio [ζ < 0.5]",
        _mean(abs.(epct_nw[lo])), _mean(abs.(epct_w[lo])))
println()
@printf("  %-34s %10.2f%%  %10.2f%%\n", "Error %% máx  [ζ ≥ 0.5]",
        maximum(abs.(epct_nw[hi])), maximum(abs.(epct_w[hi])))
@printf("  %-34s %10.2f%%  %10.2f%%\n", "Error %% medio [ζ ≥ 0.5]",
        _mean(abs.(epct_nw[hi])), _mean(abs.(epct_w[hi])))

# ──────────────────────────────────────────────────────
# 9. Verificación cruzada con distintos ωn
# ──────────────────────────────────────────────────────
println("\n" * "-" ^ 72)
println("VERIFICACIÓN CRUZADA (distintos ωn y ζ)")
println("-" ^ 72)
@printf("%6s %6s %12s %12s %10s %12s %10s\n",
        "ζ", "ωn", "tr exacto", "tr pond.", "Err pond.", "tr s/peso", "Err s/p")
println("-" ^ 72)

test_cases = [(0.1,1.0),(0.2,1.0),(0.3,1.0),(0.4,1.0),
              (0.5,1.0),(0.6,1.0),(0.7,1.0),(0.8,1.0)]

for (ζt, ωnt) in test_cases
    tr_real = rise_time_exact(ζt; ωn=ωnt)
    tr_wp   = polyeval(c_w,  [ζt])[1] / ωnt
    tr_nwp  = polyeval(c_nw, [ζt])[1] / ωnt
    ew  = 100 * abs(tr_real - tr_wp)  / tr_real
    enw = 100 * abs(tr_real - tr_nwp) / tr_real
    mark = ζt ≥ 0.5 ? " ◀" : ""
    @printf("%6.2f %6.1f %12.6f %12.6f %8.2f%% %12.6f %8.2f%%%s\n",
            ζt, ωnt, tr_real, tr_wp, ew, tr_nwp, enw, mark)
end

# ──────────────────────────────────────────────────────
# 10. Gráficas
# ──────────────────────────────────────────────────────
println("\nGenerando gráficas...")

# --- (a) Comparación directa ---
p1 = plot(ζ_fit, tr_norm; lw=2.5, color=:blue, label="Exacto (simulación)",
          xlabel="ζ", ylabel="tr·ωn", title="(a) Exacto vs ambas aproximaciones",
          xlims=(0,1), grid=true, gridalpha=0.3)
plot!(p1, ζ_fit, poly_w; lw=2, ls=:dash, color=:red, label="Ponderado (nuevo)")
plot!(p1, ζ_fit, poly_nw; lw=1.8, ls=:dot, color=:black, label="Sin ponderar (anterior)")
vspan!(p1, [valz, 1.0]; fillalpha=0.08, color=:red, label="Zona priorizada ζ≥0.5")

# --- (b) Error porcentual comparado ---
epct_w_full  = 100.0 .* (tr_norm[valid] .- poly_w[valid])  ./ tr_norm[valid]
epct_nw_full = 100.0 .* (tr_norm[valid] .- poly_nw[valid]) ./ tr_norm[valid]

p2 = plot(ζ_fit[valid], epct_w_full; lw=2, color=:red, label="Ponderado",
          xlabel="ζ", ylabel="Error [%]", title="(b) Error porcentual comparado",
          xlims=(0,1), grid=true, gridalpha=0.3)
plot!(p2, ζ_fit[valid], epct_nw_full; lw=1.5, ls=:dash, color=:black, label="Sin ponderar")
hline!(p2, [0.0]; color=:gray, lw=0.5, label=false)
vspan!(p2, [valz]; fillalpha=0.08, color=:red, label=false)

# --- (c) Respuestas al escalón ---
ζ_demo = [0.1, 0.3, 0.5, 0.7, 0.9]
t_demo = collect(range(0.0, 25.0; length=5000))
palette_demo = cgrad(:viridis, length(ζ_demo); categorical=true)

p3 = plot(xlabel="t·ωn", ylabel="y(t)", title="(c) Respuestas al escalón (ωn=1)",
          xlims=(0,25), grid=true, gridalpha=0.3)
for (i, z) in enumerate(ζ_demo)
    y = step_response(z, 1.0, t_demo)
    tr_val = rise_time_exact(z; ωn=1.0)
    plot!(p3, t_demo, y; lw=1.8, color=palette_demo[i],
          label=@sprintf("ζ=%.1f, tr=%.2f", z, tr_val))
end
hline!(p3, [0.1, 0.9]; color=:gray, ls=:dot, lw=1, alpha=0.7, label=false)

# --- (d) Zoom en zona priorizada ---
ζ_zoom   = collect(range(0.5, 0.95; length=150))
tr_zoom  = [rise_time_exact(z) for z in ζ_zoom]
pw_zoom  = polyeval(c_w, ζ_zoom)
pnw_zoom = polyeval(c_nw, ζ_zoom)

label_w  = @sprintf("Pond: %.3f%+.3fζ%+.3fζ²",  c_w[1],  c_w[2],  c_w[3])
label_nw = @sprintf("S/p:  %.3f%+.3fζ%+.3fζ²", c_nw[1], c_nw[2], c_nw[3])

p4 = plot(ζ_zoom, tr_zoom; lw=2.5, color=:blue, label="Exacto",
          xlabel="ζ", ylabel="tr·ωn", title="(d) Zoom en zona priorizada ζ∈[0.5,1]",
          grid=true, gridalpha=0.3)
plot!(p4, ζ_zoom, pw_zoom;  lw=2,   ls=:dash, color=:red,   label=label_w)
plot!(p4, ζ_zoom, pnw_zoom; lw=1.8, ls=:dot,  color=:black, label=label_nw)

# --- Combinar ---
fig = plot(p1, p2, p3, p4; layout=(2,2), size=(1400, 1000),
           plot_title="Aproximación polinomial de 2° orden para tr·ωn — Ponderación alta en ζ∈[0.5, 1]",
           plot_titlefontsize=13, margin=5Plots.mm)
display(fig)
savefig(fig, "rise_time_approximation.png")
println("\n✓ Gráfica guardada en: rise_time_approximation.png")

