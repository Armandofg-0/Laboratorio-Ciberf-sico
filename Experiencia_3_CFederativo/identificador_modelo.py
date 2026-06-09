import os
import pickle
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
from sklearn.preprocessing import StandardScaler
from nfoursid.nfoursid import NFourSID
from KF import DiscreteKalmanFilter


def preparar_datos(ruta_csv):
    data_frame = pd.read_csv(ruta_csv).drop(columns=["sim_step"], errors="ignore")
    grilla_uniforme = list(range(int(data_frame["step"].min()), int(data_frame["step"].max()) + 1))
    if len(grilla_uniforme) > len(data_frame):
        df_reindexado = data_frame.set_index("step").reindex(grilla_uniforme)
        v_cols = [c for c in df_reindexado.columns if c.startswith("v")]
        t_cols = [c for c in df_reindexado.columns if c.startswith("T") or c == "Tout"]
        df_reindexado[v_cols] = df_reindexado[v_cols].ffill()
        df_reindexado[t_cols] = df_reindexado[t_cols].interpolate(method="linear")
        df_reindexado.index.name = "step"
        df_final = df_reindexado.reset_index()
    else:
        df_final = data_frame.copy()
    volt_cols = [c for c in df_final.columns if c.startswith("v")]
    room_cols = [c for c in df_final.columns if c.startswith("T") and c != "Tout"]
    return df_final, df_final[volt_cols + ["Tout"]], df_final[room_cols]


ruta_csv = "datos_identificacion_planta_4.csv" # SOLO TIENEN QUE CAMBIAR ESTE NOMBRE PARA SACAR DATOS
Train_fraction = 0.8
_, u1, y1 = preparar_datos(ruta_csv)
u_cols, y_cols = list(u1.columns), list(y1.columns)
df_io = pd.concat([u1, y1], axis=1)
n_train = int(Train_fraction * len(df_io))
su = StandardScaler().fit(df_io[u_cols].values[:n_train])
sy = StandardScaler().fit(df_io[y_cols].values[:n_train])
U, Y = su.transform(df_io[u_cols].values), sy.transform(df_io[y_cols].values)
U_tr, U_val = U[:n_train], U[n_train:]
Y_tr, Y_val = Y[:n_train], Y[n_train:]
df_tr = pd.DataFrame(np.hstack([U_tr, Y_tr]), columns=u_cols + y_cols)
i = 40
modelo_n4sid = NFourSID(df_tr, input_columns=u_cols, output_columns=y_cols, num_block_rows=i)
modelo_n4sid.subspace_identification()
orden = 7
modelo_ee, cov = modelo_n4sid.system_identification(rank=orden)
A, B, C, D = modelo_ee.a, modelo_ee.b, modelo_ee.c, modelo_ee.d
ny = len(y_cols)
R, Q = cov[:ny, :ny], cov[ny:, ny:]
nx, nu = A.shape[0], B.shape[1]

################################################################################
plant_id = ruta_csv.split("planta_")[1].split(".")[0]
estimacion = {
    "A": np.asarray(A, dtype=float), "B": np.asarray(B, dtype=float),
    "C": np.asarray(C, dtype=float), "D": np.asarray(D, dtype=float),
    "Q": np.asarray(Q, dtype=float), "R": np.asarray(R, dtype=float),
    "u_mean": su.mean_, "u_std": su.scale_,   # entradas del modelo: [v1..v6, Tout]
    "y_mean": sy.mean_, "y_std": sy.scale_,   # salidas del modelo: T de las 6 habitaciones
    "u_cols": u_cols, "y_cols": y_cols,       # orden de las variables, para armar bien los vectores
}
os.makedirs("resultados", exist_ok=True)
with open(f"resultados/estimacion_planta_{plant_id}.pkl", "wb") as f:
    pickle.dump(estimacion, f)
print(f"Estimación guardada en resultados/estimacion_planta_{plant_id}.pkl")
################################################################################

L = 15
H = L - 1
n_win = len(U_val) - L + 1
pred, real = np.zeros((n_win, H, ny)), np.zeros((n_win, H, ny))
for ventana in range(n_win):
    u_w, y_w = U_val[ventana:ventana + L], Y_val[ventana:ventana + L]
    y0, u0 = y_w[0].reshape(ny, 1), u_w[0].reshape(nu, 1)
    x0 = np.linalg.pinv(C) @ (y0 - D @ u0)
    kf = DiscreteKalmanFilter.create(A=A, B=B, C=C, D=D, Q=Q, R=R, P_init=np.eye(nx), x_init=x0)
    kf.update(y0, u0)
    for h in range(H):
        kf.predict(u_w[h].reshape(nu, 1))
        pred[ventana, h] = kf.output_prediction(u_w[h + 1].reshape(nu, 1)).ravel()
        real[ventana, h] = y_w[h + 1]
pred_C, real_C = pred * sy.scale_ + sy.mean_, real * sy.scale_ + sy.mean_
error = pred_C - real_C
rmse_paso = np.sqrt(np.mean(error ** 2, axis=(0, 2)))
mape_paso = 100 * np.mean(np.abs(error) / np.abs(real_C).clip(min=0.5), axis=(0, 2))
print(f"{'Paso':>5} | {'RMSE (°C)':>10} | {'MAPE (%)':>9}")
print("-" * 32)
for h in range(H):
    print(f"  t+{h + 1:<2} | {rmse_paso[h]:>10.3f} | {mape_paso[h]:>9.2f}")
print("-" * 32)
print(f"  MEDIA | {rmse_paso.mean():>9.3f} | {mape_paso.mean():>9.2f}")
mae_win = np.mean(np.abs(error), axis=(1, 2))
s_ej = int(np.argmin(mae_win))
horizonte = np.arange(1, H + 1)
fig, axes = plt.subplots(2, 3, figsize=(13, 6), sharex=True)
for r, ax in enumerate(axes.ravel()):
    ax.plot(horizonte, real_C[s_ej, :, r], 'o-', label='Real')
    ax.plot(horizonte, pred_C[s_ej, :, r], 's--', label='Estimada')
    ax.set_title(f'Habitación {r + 1}')
    ax.set_xlabel('Minutos al futuro')
    ax.set_ylabel('Temperatura (°C)')
    ax.grid(alpha=0.3)
axes[0, 0].legend()
fig.suptitle(f'Predicción libre a 14 pasos — mejor ventana (s={s_ej}, MAE={mae_win[s_ej]:.3f} °C)')
plt.tight_layout()
plt.show()