import os, pickle, numpy as np, torch
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


def entrenar(model, opt, At, Bt, Ct, Dt, x, yc, v, rn, tout, umean, ustd, H, Wy, Wu, Wf):
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
    J = J + Wf * ((yc - rn) ** 2).sum()
    J.backward(); opt.step()
    return J.item(), v0.detach().cpu().numpy()


def control(plant_id=2, r_setpoint=22.0, sala=0, H=14, hidden=128, lr=1e-3, Wy=1.0, Wu=0.01, Wf=5.0, every=10):
    with open(f"results/estimacion_planta_{plant_id}.pkl", "rb") as f: est = pickle.load(f)
    A, B, C, D, Q, R = est["A"], est["B"], est["C"], est["D"], est["Q"], est["R"]
    u_mean, u_std, y_mean, y_std = est["u_mean"], est["u_std"], est["y_mean"], est["y_std"]
    nx, ny, nu = A.shape[0], C.shape[0], B.shape[1]; nv = nu - 1
    dev = torch.device('cuda' if torch.cuda.is_available() else 'cpu')
    t = lambda M: torch.from_numpy(np.asarray(M, dtype=float)).float().to(dev)
    At, Bt, Ct, Dt, umean_t, ustd_t = t(A), t(B), t(C), t(D), t(u_mean), t(u_std)
    rn = (torch.full((ny,), r_setpoint, device=dev) - t(y_mean)) / t(y_std)
    model = ControladorNeuronal(ny + nx + nv, nv, hidden).to(dev)
    opt = optim.Adam(model.parameters(), lr=lr)
    kalman = DiscreteKalmanFilter.create(A=A, B=B, C=C, Q=Q, R=R, P_init=np.eye(nx), D=D, x_init=np.zeros((nx, 1)))
    v_prev, losses, temps, iae, n = np.zeros(nv), [], [], 0.0, 0
    client = MyClient("opc.tcp://192.168.1.142:4840/freeopcua/server/"); client.connect()
    step = int(client.sim_step.get_value())
    try:
        while True:
            step = client.wait_for_next_step(step)
            y = np.array([client.T_rooms[f'T_room{i+1}'].get_value() for i in range(ny)])
            tout = client.T_outdoor.get_value()
            y_n = (y - y_mean) / y_std
            u_prev = (np.concatenate([v_prev, [tout]]) - u_mean) / u_std
            kalman.step(u_prev.reshape(nu, 1), y_n.reshape(ny, 1))
            loss, v0 = entrenar(model, opt, At, Bt, Ct, Dt, t(kalman.x.flatten()), t(y_n), t(v_prev),
                                rn, t(np.array([tout])), umean_t, ustd_t, H, Wy, Wu, Wf)
            for i in range(nv): client.voltages[f'Volt_room{i+1}'].set_value(float(v0[i]))
            v_prev = v0
            iae += float(np.abs(r_setpoint - y).sum())
            losses.append(loss); temps.append(float(y[sala]))
            if n % every == 0:
                print(f"[{step}] T{sala+1}={y[sala]:.2f}°C ref={r_setpoint:.1f} loss={loss:.4f} IAE={iae:.1f}", end="\r", flush=True)
            n += 1
    finally:
        client.disconnect()
        os.makedirs("results", exist_ok=True)
        with open(f"results/control_planta_{plant_id}.pkl", "wb") as f:
            pickle.dump({"loss": np.array(losses), "temp": np.array(temps), "ref": r_setpoint, "sala": sala, "iae": iae}, f)


def main():
    control(plant_id=2)


if __name__ == "__main__":
    main()