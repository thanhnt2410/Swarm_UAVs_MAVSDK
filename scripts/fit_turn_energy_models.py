#!/usr/bin/env python3
"""Fit interpretable turn energy/time models from collected SITL data."""

from __future__ import annotations

import argparse
import math
from pathlib import Path
from typing import Dict, Iterable, Optional, Sequence, Tuple

import numpy as np
import pandas as pd


def design_matrix(theta: np.ndarray, degree: int) -> np.ndarray:
    columns = [theta ** power for power in range(1, degree + 1)]
    columns.append(np.ones_like(theta))
    return np.column_stack(columns)


def fit_polynomial(theta: np.ndarray, y: np.ndarray, degree: int) -> np.ndarray:
    x = design_matrix(theta, degree)
    coef, *_ = np.linalg.lstsq(x, y, rcond=None)
    return coef


def predict_polynomial(theta: np.ndarray, coef: np.ndarray) -> np.ndarray:
    degree = len(coef) - 1
    return design_matrix(theta, degree) @ coef


def metrics(y_true: np.ndarray, y_pred: np.ndarray) -> Dict[str, float]:
    err = y_pred - y_true
    mae = float(np.mean(np.abs(err)))
    rmse = float(math.sqrt(np.mean(err ** 2)))
    finite = np.abs(y_true) > 1e-12
    mape = float(np.mean(np.abs(err[finite] / y_true[finite])) * 100.0) if np.any(finite) else float("nan")
    denom = float(np.sum((y_true - np.mean(y_true)) ** 2))
    r2 = 1.0 - float(np.sum(err ** 2)) / denom if denom > 1e-12 else float("nan")
    return {"MAE": mae, "RMSE": rmse, "MAPE_percent": mape, "R2": r2}


def split_by_repeat(df: pd.DataFrame) -> Tuple[pd.DataFrame, pd.DataFrame]:
    if "repeat_index" not in df.columns or df["repeat_index"].nunique() < 2:
        return df, df
    all_angles = set(df["angle_deg"].unique())
    complete_repeats = [
        rep for rep, group in df.groupby("repeat_index")
        if set(group["angle_deg"].unique()) == all_angles
    ]
    if len(complete_repeats) >= 2:
        test_repeat = max(complete_repeats)
        train_repeats = [r for r in complete_repeats if r != test_repeat]
        train = df[df["repeat_index"].isin(train_repeats)]
        test = df[df["repeat_index"] == test_repeat]
        return train, test
    max_repeat = df["repeat_index"].max()
    test = df[df["repeat_index"] == max_repeat]
    train = df[df["repeat_index"] != max_repeat]
    return train, test


def format_formula(target: str, coef: np.ndarray) -> str:
    degree = len(coef) - 1
    terms = []
    for idx, value in enumerate(coef[:-1], start=1):
        if degree == 1:
            terms.append(f"{value:.8g} * abs(theta_rad)")
        else:
            terms.append(f"{value:.8g} * abs(theta_rad)^{idx}")
    terms.append(f"{coef[-1]:.8g}")
    return f"{target} = " + " + ".join(terms)


def fit_target(train: pd.DataFrame, test: pd.DataFrame, target: str, degree: int) -> Tuple[np.ndarray, Dict[str, float]]:
    theta_train = np.abs(train["angle_rad"].to_numpy(dtype=float))
    y_train = train[target].to_numpy(dtype=float)
    theta_test = np.abs(test["angle_rad"].to_numpy(dtype=float))
    y_test = test[target].to_numpy(dtype=float)
    coef = fit_polynomial(theta_train, y_train, degree)
    y_pred = predict_polynomial(theta_test, coef)
    return coef, metrics(y_test, y_pred)


def maybe_fit_random_forest(train: pd.DataFrame, test: pd.DataFrame, target: str) -> Optional[Dict[str, float]]:
    try:
        from sklearn.ensemble import RandomForestRegressor
    except Exception:
        return None

    features = ["angle_rad", "speed_m_s", "total_distance_m"]
    train_features = train[features].copy()
    test_features = test[features].copy()
    train_features["angle_rad"] = train_features["angle_rad"].abs()
    test_features["angle_rad"] = test_features["angle_rad"].abs()
    model = RandomForestRegressor(n_estimators=200, random_state=42, min_samples_leaf=1)
    model.fit(train_features, train[target])
    pred = model.predict(test_features)
    return metrics(test[target].to_numpy(dtype=float), pred)


def main(argv: Optional[Sequence[str]] = None) -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("dataset", type=Path)
    parser.add_argument("--degree", type=int, default=2, choices=(1, 2))
    parser.add_argument("--include-invalid", action="store_true")
    args = parser.parse_args(argv)

    df = pd.read_csv(args.dataset)
    if not args.include_invalid and "valid" in df.columns:
        df = df[df["valid"].astype(bool)]
    df = df.replace("", np.nan).dropna(subset=["angle_rad", "turn_energy_wh", "turn_time_s"])
    if df.empty:
        raise SystemExit("No valid rows with turn_energy_wh and turn_time_s were found.")

    train, test = split_by_repeat(df)
    print(f"Rows: train={len(train)}, test={len(test)}, total={len(df)}")
    print(f"Angles: {sorted(df['angle_deg'].unique().tolist())}")

    for target in ("turn_energy_wh", "turn_time_s"):
        print(f"\n[{target}] polynomial degree {args.degree}")
        coef, result_metrics = fit_target(train, test, target, args.degree)
        print(format_formula(target, coef))
        for name, value in result_metrics.items():
            print(f"{name}: {value:.6g}")

        rf_metrics = maybe_fit_random_forest(train, test, target)
        if rf_metrics is None:
            print("RandomForest benchmark: skipped (scikit-learn is not installed)")
        else:
            print("RandomForest benchmark:")
            for name, value in rf_metrics.items():
                print(f"  {name}: {value:.6g}")


if __name__ == "__main__":
    main()
