import numpy as np
from dataclasses import dataclass
from typing import Optional, Dict


def _sym(M: np.ndarray) -> np.ndarray:
    return 0.5 * (M + M.T)


@dataclass
class DiscreteKalmanFilter:
    """
    Discrete-time Kalman Filter for:
        x_{k+1} = A x_k + B u_k + w_k
        y_k     = C x_k + D u_k + v_k

    with:
        w ~ N(0, Q), v ~ N(0, R)
    """
    A: np.ndarray
    B: np.ndarray
    C: np.ndarray
    D: Optional[np.ndarray]
    Q: np.ndarray
    R: np.ndarray
    x: np.ndarray
    P: np.ndarray
    eps: float = 1e-12

    @classmethod
    def create(
        cls,
        A: np.ndarray,
        B: np.ndarray,
        C: np.ndarray,
        Q: np.ndarray,
        R: np.ndarray,
        P_init: np.ndarray,
        D: Optional[np.ndarray] = None,
        x_init: Optional[np.ndarray] = None,
        eps: float = 1e-12,
    ) -> "DiscreteKalmanFilter":
        A = np.asarray(A, dtype=float)
        B = np.asarray(B, dtype=float)
        C = np.asarray(C, dtype=float)
        Q = np.asarray(Q, dtype=float)
        R = np.asarray(R, dtype=float)
        P_init = np.asarray(P_init, dtype=float)

        nx = A.shape[0]
        nu = B.shape[1]
        ny = C.shape[0]

        if A.shape != (nx, nx):
            raise ValueError(f"A must be (nx,nx); got {A.shape}")
        if B.shape[0] != nx:
            raise ValueError(f"B must be (nx,nu); got {B.shape}")
        if C.shape[1] != nx:
            raise ValueError(f"C must be (ny,nx); got {C.shape}")
        if Q.shape != (nx, nx):
            raise ValueError(f"Q must be (nx,nx); got {Q.shape}")
        if R.shape != (ny, ny):
            raise ValueError(f"R must be (ny,ny); got {R.shape}")
        if P_init.shape != (nx, nx):
            raise ValueError(f"P_init must be (nx,nx); got {P_init.shape}")

        if D is None:
            D = np.zeros((ny, nu), dtype=float)
        else:
            D = np.asarray(D, dtype=float)
            if D.shape != (ny, nu):
                raise ValueError(f"D must be (ny,nu); got {D.shape}")

        if x_init is None:
            x_init = np.zeros((nx, 1), dtype=float)
        else:
            x_init = np.asarray(x_init, dtype=float)
            if x_init.shape == (nx,):
                x_init = x_init.reshape(nx, 1)
            if x_init.shape != (nx, 1):
                raise ValueError(f"x_init must be (nx,1); got {x_init.shape}")

        Q = _sym(Q)
        R = _sym(R)
        P_init = _sym(P_init)

        return cls(A=A, B=B, C=C, D=D, Q=Q, R=R, x=x_init, P=P_init, eps=eps)

    def predict(self, u: np.ndarray) -> Dict[str, np.ndarray]:
        """
        Time update:
            x^- = A x + B u
            P^- = A P A^T + Q
        """
        u = np.asarray(u, dtype=float)
        nu = self.B.shape[1]

        if u.shape == (nu,):
            u = u.reshape(nu, 1)
        if u.shape != (nu, 1):
            raise ValueError(f"u must be (nu,1); got {u.shape}")

        x_pred = self.A @ self.x + self.B @ u
        P_pred = self.A @ self.P @ self.A.T + self.Q
        P_pred = _sym(P_pred)

        self.x = x_pred
        self.P = P_pred

        return {"x_pred": x_pred, "P_pred": P_pred}

    def update(self, y: np.ndarray, u: Optional[np.ndarray] = None) -> Dict[str, np.ndarray]:
        """
        Measurement update (Joseph form):
            y_hat = C x^- + D u
            S = C P^- C^T + R
            K = P^- C^T S^{-1}
            x^+ = x^- + K (y - y_hat)
            P^+ = (I - K C) P^- (I - K C)^T + K R K^T
        """
        y = np.asarray(y, dtype=float)
        ny = self.C.shape[0]

        if y.shape == (ny,):
            y = y.reshape(ny, 1)
        if y.shape != (ny, 1):
            raise ValueError(f"y must be (ny,1); got {y.shape}")

        if u is None:
            u = np.zeros((self.B.shape[1], 1), dtype=float)
        else:
            u = np.asarray(u, dtype=float)
            nu = self.B.shape[1]
            if u.shape == (nu,):
                u = u.reshape(nu, 1)
            if u.shape != (nu, 1):
                raise ValueError(f"u must be (nu,1); got {u.shape}")

        y_hat = self.C @ self.x + self.D @ u

        S = self.C @ self.P @ self.C.T + self.R
        S = _sym(S)

        S = S + self.eps * np.eye(S.shape[0])

        K = self.P @ self.C.T @ np.linalg.solve(S, np.eye(S.shape[0]))

        innov = y - y_hat
        x_filt = self.x + K @ innov

        I = np.eye(self.P.shape[0])
        IKC = I - K @ self.C

        P_filt = IKC @ self.P @ IKC.T + K @ self.R @ K.T
        P_filt = _sym(P_filt)

        # Store filtered
        self.x = x_filt
        self.P = P_filt

        return {
            "x_filt": x_filt,
            "P_filt": P_filt,
            "K": K,
            "innov": innov,
            "y_hat": y_hat,
            "S": S,
        }

    def step(
        self,
        u: np.ndarray,
        y: Optional[np.ndarray] = None,
    ) -> Dict[str, np.ndarray]:

        out = {}
        out.update(self.predict(u))

        if y is not None:
            out.update(self.update(y, u=u))
        return out

    def output_prediction(self, u: Optional[np.ndarray] = None) -> np.ndarray:

        ny = self.C.shape[0]
        nu = self.B.shape[1]
        if u is None:
            u = np.zeros((nu, 1))
        else:
            u = np.asarray(u, dtype=float)
            if u.shape == (nu,):
                u = u.reshape(nu, 1)
        return self.C @ self.x + self.D @ u
