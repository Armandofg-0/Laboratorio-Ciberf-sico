import pickle, numpy as np, torch
import torch.nn as nn, torch.nn.functional as F, torch.optim as optim
from KF import DiscreteKalmanFilter
from buildingClient import MyClient


class ControladorNeuronal(nn.Module):
    def __init__(self, input_size, output_size=6, hidden_size=128, dropout=0.3):
        super().__init__()
        self.l_in = nn.Linear(input_size, hidden_size)
        self.l_h1 = nn.Linear(hidden_size, hidden_size)
        self.l_h2 = nn.Linear(hidden_size, hidden_size)
        self.l_h3 = nn.Linear(hidden_size, hidden_size // 2)
        self.l_out = nn.Linear(hidden_size // 2, output_size)
        self.drop = nn.Dropout(dropout)

    def forward(self, x):
        x = self.l_in(x)
        x = self.drop(F.relu(self.l_h1(x)))
        x = self.drop(F.relu(self.l_h2(x)))
        x = self.drop(F.relu(self.l_h3(x)))
        return torch.tanh(self.l_out(x))


def costo_terminal_P(A, B, C, D, Wy, Wu, iters=500_000, tol=1e-9):
    """Costo-por-ir de horizonte infinito: solución de la ecuación de Riccati discreta (DARE).
    Tout es perturbación fija: solo las primeras nv columnas de B y D son manipulables."""
    nv = B.shape[1] - 1
    Bv, Dv = B[:, :nv], D[:, :nv]
    ny = C.shape[0]
    Qm = C.T @ (Wy * np.eye(ny)) @ C
    Rm = Dv.T @ (Wy * np.eye(ny)) @ Dv + Wu * np.eye(nv)
    Nm = C.T @ (Wy * np.eye(ny)) @ Dv
    P = Qm.copy()
    for _ in range(iters):
        K = np.linalg.solve(Rm + Bv.T @ P @ Bv, Bv.T @ P @ A + Nm.T)
        Pn = Qm + A.T @ P @ A - (A.T @ P @ Bv + Nm) @ K
        Pn = 0.5 * (Pn + Pn.T)
        if np.max(np.abs(Pn - P)) < tol:
            return Pn
        P = Pn
    return P


def estado_equilibrio(A, B, C, D, u_mean, u_std, y_mean, y_std, r_setpoint, tout):
    """Estado x* y voltaje v* (físico) que sostienen y=r con la Tout dada."""
    nx = A.shape[0]
    nv = B.shape[1] - 1
    Mdc = C @ np.linalg.solve(np.eye(nx) - A, B) + D
    rn_ = (np.full(C.shape[0], r_setpoint) - y_mean) / y_std
    tout_n = (tout - u_mean[-1]) / u_std[-1]
    v_n = np.linalg.solve(Mdc[:, :nv], rn_ - Mdc[:, -1] * tout_n)
    u_n = np.concatenate([v_n, [tout_n]])
    xstar = np.linalg.solve(np.eye(nx) - A, B @ u_n)
    vstar = v_n * u_std[:nv] + u_mean[:nv]          # voltaje de equilibrio en unidades físicas
    return xstar, vstar


def entrenar(model, opt, At, Bt, Ct, Dt, x, yc, v, rn, tout, umean, ustd, H, Wy, Wu, xstar, P):
    opt.zero_grad()
    J, v0 = 0.0, None
    for k in range(H):
        vn = model(torch.cat([rn - yc, x, v]))
        u = (torch.cat([vn, tout]) - umean) / ustd
        x = At @ x + Bt @ u
        yc = Ct @ x + Dt @ u
        J = J + Wy * ((yc - rn) ** 2).sum() + Wu * ((vn - v) ** 2).sum()
        if k == 0: v0 = vn
        v = vn
    e = x - xstar
    J = J + e @ P @ e
    J.backward()
    torch.nn.utils.clip_grad_norm_(model.parameters(), max_norm=1.0)
    opt.step()
    return J.item(), v0.detach().cpu().numpy()


def control(plant_id=2, ip="192.168.1.142", r_setpoint=22.0, sala=0, H=14, hidden=128, lr=1e-3, Wy=1.0, Wu=0.05, every=10, Ki=5e-3, b_max=2.0):
    with open(f"estimacion_planta_{plant_id}.pkl", "rb") as f: est = pickle.load(f)
    A, B, C, D, Q, R = (np.asarray(est[k]) for k in ["A", "B", "C", "D", "Q", "R"])
    u_mean, u_std, y_mean, y_std = est["u_mean"], est["u_std"], est["y_mean"], est["y_std"]
    nx, ny, nu = A.shape[0], C.shape[0], B.shape[1]; nv = nu - 1
    dev = torch.device('cuda' if torch.cuda.is_available() else 'cpu')
    t = lambda M: torch.from_numpy(np.asarray(M, dtype=float)).float().to(dev)
    At, Bt, Ct, Dt, umean_t, ustd_t = t(A), t(B), t(C), t(D), t(u_mean), t(u_std)
    ymean_t, ystd_t = t(y_mean), t(y_std)  # rn ahora se recalcula en el loop con la referencia corregida
    Pt = t(costo_terminal_P(A, B, C, D, Wy, Wu))  # costo terminal: se calcula una vez
    model = ControladorNeuronal(ny + nx + nv, nv, hidden).to(dev)
    opt = optim.Adam(model.parameters(), lr=lr)
    kalman = DiscreteKalmanFilter.create(A=A, B=B, C=C, Q=Q, R=R, P_init=np.eye(nx), D=D, x_init=np.zeros((nx, 1)))
    v_prev, losses, temps, iae, n = np.zeros(nv), [], [], 0.0, 0
    b = np.zeros(ny)  # sesgo integral por habitación (corrección de error de modelo en DC)
    port = 4838 + plant_id  # 4840/4841/4842 para plantas 2/3/4
    client = MyClient(f"opc.tcp://{ip}:{port}/freeopcua/server/"); client.connect()
    step = int(client.sim_step.get_value())
    try:
        while True:
            step = client.wait_for_next_step(step)
            y = np.array([client.T_rooms[f'T_room{i+1}'].get_value() for i in range(ny)])
            tout = client.T_outdoor.get_value()
            y_n = (y - y_mean) / y_std
            u_prev = (np.concatenate([v_prev, [tout]]) - u_mean) / u_std
            kalman.step(u_prev.reshape(nu, 1), y_n.reshape(ny, 1))
            b = np.clip(b + Ki * (r_setpoint - y), -b_max, b_max)  # integrador con anti-windup
            r_eff = r_setpoint + b
            rn = (t(r_eff) - ymean_t) / ystd_t
            xstar_t = t(estado_equilibrio(A, B, C, D, u_mean, u_std, y_mean, y_std, r_eff, tout)[0])
            loss, _ = entrenar(model, opt, At, Bt, Ct, Dt, t(kalman.x.flatten()), t(y_n), t(v_prev),
                               rn, t(np.array([tout])), umean_t, ustd_t, H, Wy, Wu, xstar_t, Pt)
            model.eval()
            with torch.no_grad():
                v0 = model(torch.cat([rn - t(y_n), t(kalman.x.flatten()), t(v_prev)])).cpu().numpy()
            model.train()
            for i in range(nv): client.voltages[f'Volt_room{i+1}'].set_value(float(v0[i]))
            v_prev = v0
            iae += float(np.abs(r_setpoint - y).sum())
            losses.append(loss); temps.append(float(y[sala]))
            if n % every == 0:
                print(f"[{step}] T{sala+1}={y[sala]:.2f}°C ref={r_setpoint:.1f} b={b[sala]:+.2f} loss={loss:.4f} IAE={iae:.1f}", end="\r", flush=True)
            n += 1
    finally:
        client.disconnect()
        with open(f"control_planta_{plant_id}.pkl", "wb") as f:
            pickle.dump({"loss": np.array(losses), "temp": np.array(temps), "ref": r_setpoint, "sala": sala, "iae": iae}, f)


def main():
    import sys
    control(plant_id=int(sys.argv[1]) if len(sys.argv) > 1 else 2)


if __name__ == "__main__":
    main()