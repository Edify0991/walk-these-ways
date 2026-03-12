import argparse
import pickle as pkl
from pathlib import Path

import numpy as np


def _normalize(x: np.ndarray) -> np.ndarray:
    x = np.asarray(x).astype(np.float64)
    x = x - x.mean()
    std = x.std()
    if std < 1e-8:
        return np.zeros_like(x)
    return x / std


def estimate_lag_steps(signal_a: np.ndarray, signal_b: np.ndarray, max_lag: int) -> tuple[int, float]:
    """
    Find lag k in [-max_lag, max_lag] maximizing corr(a[t-k], b[t]).
    Positive lag => a leads b (b is delayed).
    """
    a = _normalize(signal_a)
    b = _normalize(signal_b)

    best_lag = 0
    best_corr = -np.inf

    for lag in range(-max_lag, max_lag + 1):
        if lag >= 0:
            aa = a[:-lag] if lag > 0 else a
            bb = b[lag:]
        else:
            aa = a[-lag:]
            bb = b[:lag]

        if len(aa) < 20:
            continue

        corr = float(np.mean(aa * bb))
        if corr > best_corr:
            best_corr = corr
            best_lag = lag

    return best_lag, best_corr


def load_series(log_path: Path):
    with open(log_path, 'rb') as f:
        data = pkl.load(f)

    entries = data['hardware_closed_loop'][1]
    if len(entries) < 10:
        raise ValueError('Not enough samples in hardware_closed_loop log.')

    required = ['joint_pos', 'joint_pos_target', 'joint_vel', 'torques']
    for k in required:
        if k not in entries[0]:
            raise KeyError(f"Missing key '{k}' in log entries.")

    joint_pos = np.array([e['joint_pos'] for e in entries], dtype=np.float64)
    joint_pos_target = np.array([e['joint_pos_target'] for e in entries], dtype=np.float64)
    joint_vel = np.array([e['joint_vel'] for e in entries], dtype=np.float64)
    torques = np.array([e['torques'] for e in entries], dtype=np.float64)
    tau_est = None
    if 'tau_est' in entries[0]:
        tau_est = np.array([e['tau_est'] for e in entries], dtype=np.float64)

    return joint_pos, joint_pos_target, joint_vel, torques, tau_est


def main():
    parser = argparse.ArgumentParser(description='Estimate system latency from a hardware log.pkl')
    parser.add_argument('--log', required=True, help='Path to log.pkl')
    parser.add_argument('--dt', type=float, default=0.02, help='Control period in seconds (default 0.02)')
    parser.add_argument('--max-lag-steps', type=int, default=12, help='Search lag range [-N, N] in steps')
    parser.add_argument('--signal', choices=['torques_to_tau_est', 'torques_to_pos_err'], default='torques_to_tau_est',
                        help='Latency metric to use')
    args = parser.parse_args()

    log_path = Path(args.log)
    joint_pos, joint_pos_target, joint_vel, torques, tau_est = load_series(log_path)

    if args.signal == 'torques_to_tau_est' and tau_est is None:
        raise ValueError('signal=torques_to_tau_est requires tau_est in the log.')

    # per-joint estimate
    lags = []
    corrs = []
    for j in range(12):
        cmd = torques[:, j]
        if args.signal == 'torques_to_tau_est':
            resp = tau_est[:, j]
        else:
            # proxy response: position error dynamics
            pos_err = joint_pos[:, j] - joint_pos_target[:, j]
            resp = np.gradient(pos_err)

        lag, corr = estimate_lag_steps(cmd, resp, args.max_lag_steps)
        lags.append(lag)
        corrs.append(corr)

    lags = np.array(lags)
    corrs = np.array(corrs)

    median_lag = int(np.median(lags))
    mean_lag = float(np.mean(lags))
    latency_ms = median_lag * args.dt * 1000.0

    print('=== Latency identification result ===')
    print(f'log: {log_path}')
    print(f'signal mode: {args.signal}')
    print(f'per-joint lag steps: {lags.tolist()}')
    print(f'per-joint corr: {[round(c, 3) for c in corrs.tolist()]}')
    print(f'median lag: {median_lag} steps')
    print(f'mean lag: {mean_lag:.2f} steps')
    print(f'suggested constant delay: {latency_ms:.1f} ms (median_lag * dt)')
    print(f'suggested cfg.domain_rand.lag_timesteps = {max(median_lag, 0)}')


if __name__ == '__main__':
    main()
