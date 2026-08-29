#!/usr/bin/env python3
"""Train and compare mission-level energy/time regression models."""

from __future__ import annotations

import argparse
import json
import math
import pickle
from pathlib import Path
from typing import Dict, List, Optional, Sequence, Tuple

import numpy as np
import pandas as pd
from sklearn.compose import TransformedTargetRegressor
from sklearn.ensemble import ExtraTreesRegressor, GradientBoostingRegressor, RandomForestRegressor
from sklearn.gaussian_process import GaussianProcessRegressor
from sklearn.gaussian_process.kernels import ConstantKernel, RBF, WhiteKernel
from sklearn.linear_model import LinearRegression, Ridge
from sklearn.metrics import mean_absolute_error, mean_squared_error, r2_score
from sklearn.model_selection import GroupShuffleSplit
from sklearn.neighbors import KNeighborsRegressor
from sklearn.pipeline import Pipeline
from sklearn.preprocessing import PolynomialFeatures, StandardScaler
from sklearn.svm import SVR
from sklearn.tree import DecisionTreeRegressor


# ---------------------------<Training defaults>---------------------------

DATASET_PATH = Path("logs/mission_energy/random_missions.csv")
OUTPUT_DIR = Path("models/mission_energy")
FEATURE_COLUMNS = ["total_distance_m", "stop_count"]
TARGET_COLUMNS = ["total_flight_energy_wh", "total_flight_time_s"]
GROUP_COLUMN = "geometry_id"
TEST_SIZE = 0.20
RANDOM_SEED = 42

RANDOM_FOREST_TREES = 500
GRADIENT_BOOSTING_TREES = 300


def parse_valid_column(series: pd.Series) -> pd.Series:
    if pd.api.types.is_bool_dtype(series):
        return series
    return series.astype(str).str.strip().str.lower().isin({"true", "1", "yes"})


def load_dataset(path: Path) -> pd.DataFrame:
    dataframe = pd.read_csv(path)
    required_columns = set(FEATURE_COLUMNS + TARGET_COLUMNS + [GROUP_COLUMN])
    missing_columns = sorted(required_columns.difference(dataframe.columns))
    if missing_columns:
        raise ValueError(f"Dataset is missing required columns: {', '.join(missing_columns)}")

    if "valid" in dataframe.columns:
        dataframe = dataframe[parse_valid_column(dataframe["valid"])]
    dataframe = dataframe.replace([np.inf, -np.inf], np.nan)
    dataframe = dataframe.dropna(subset=list(required_columns)).copy()
    if dataframe.empty:
        raise ValueError("Dataset has no valid rows after filtering")
    if dataframe[GROUP_COLUMN].nunique() < 5:
        raise ValueError("At least five unique geometry_id values are required for a group split")
    return dataframe.reset_index(drop=True)


def split_by_geometry(dataframe: pd.DataFrame) -> Tuple[pd.DataFrame, pd.DataFrame]:
    splitter = GroupShuffleSplit(n_splits=1, test_size=TEST_SIZE, random_state=RANDOM_SEED)
    train_indices, test_indices = next(
        splitter.split(dataframe, groups=dataframe[GROUP_COLUMN])
    )
    train = dataframe.iloc[train_indices].copy()
    test = dataframe.iloc[test_indices].copy()

    overlapping_groups = set(train[GROUP_COLUMN]).intersection(test[GROUP_COLUMN])
    if overlapping_groups:
        raise RuntimeError(f"Group leakage detected: {sorted(overlapping_groups)}")
    return train, test


def build_models() -> Dict[str, object]:
    try:
        from xgboost import XGBRegressor
    except ImportError as exc:
        raise RuntimeError(
            "XGBoost is required. Install project requirements before training: "
            "pip install -r requirements.txt"
        ) from exc

    return {
        "linear": Pipeline([
            ("scale", StandardScaler()),
            ("model", LinearRegression()),
        ]),
        "polynomial_degree_2": Pipeline([
            ("poly", PolynomialFeatures(degree=2, include_bias=False)),
            ("scale", StandardScaler()),
            ("model", Ridge(alpha=1.0)),
        ]),
        "svr_rbf": TransformedTargetRegressor(
            regressor=Pipeline([
                ("scale", StandardScaler()),
                ("model", SVR(kernel="rbf", C=10.0, epsilon=0.05, gamma="scale")),
            ]),
            transformer=StandardScaler(),
        ),
        "knn": Pipeline([
            ("scale", StandardScaler()),
            ("model", KNeighborsRegressor(n_neighbors=5, weights="distance", p=2)),
        ]),
        "gaussian_process": Pipeline([
            ("scale", StandardScaler()),
            ("model", GaussianProcessRegressor(
                kernel=ConstantKernel(1.0, (1e-3, 1e3))
                * RBF(length_scale=1.0, length_scale_bounds=(1e-2, 1e2))
                + WhiteKernel(noise_level=1e-3, noise_level_bounds=(1e-8, 1e1)),
                normalize_y=True,
                n_restarts_optimizer=3,
                random_state=RANDOM_SEED,
            )),
        ]),
        "decision_tree": DecisionTreeRegressor(
            max_depth=6, min_samples_leaf=2, random_state=RANDOM_SEED
        ),
        "random_forest": RandomForestRegressor(
            n_estimators=RANDOM_FOREST_TREES,
            min_samples_leaf=2,
            random_state=RANDOM_SEED,
            n_jobs=-1,
        ),
        "extra_trees": ExtraTreesRegressor(
            n_estimators=RANDOM_FOREST_TREES,
            min_samples_leaf=2,
            random_state=RANDOM_SEED,
            n_jobs=-1,
        ),
        "gradient_boosting": GradientBoostingRegressor(
            n_estimators=GRADIENT_BOOSTING_TREES,
            learning_rate=0.03,
            max_depth=3,
            loss="squared_error",
            random_state=RANDOM_SEED,
        ),
        "xgboost": XGBRegressor(
            objective="reg:squarederror",
            n_estimators=500,
            learning_rate=0.03,
            max_depth=3,
            min_child_weight=2,
            subsample=0.9,
            colsample_bytree=1.0,
            reg_lambda=1.0,
            random_state=RANDOM_SEED,
            n_jobs=-1,
        ),
    }


def regression_metrics(y_true: np.ndarray, y_predicted: np.ndarray) -> Dict[str, float]:
    absolute_error = np.abs(y_predicted - y_true)
    nonzero = np.abs(y_true) > 1e-12
    mape = float(np.mean(absolute_error[nonzero] / np.abs(y_true[nonzero])) * 100.0)
    return {
        "mae": float(mean_absolute_error(y_true, y_predicted)),
        "rmse": float(math.sqrt(mean_squared_error(y_true, y_predicted))),
        "mape_percent": mape,
        "r2": float(r2_score(y_true, y_predicted)),
    }


def train_target(
    train: pd.DataFrame,
    test: pd.DataFrame,
    target: str,
) -> Tuple[List[dict], Dict[str, object], pd.DataFrame]:
    x_train = train[FEATURE_COLUMNS]
    x_test = test[FEATURE_COLUMNS]
    y_train = train[target].to_numpy(dtype=float)
    y_test = test[target].to_numpy(dtype=float)
    trained_models: Dict[str, object] = {}
    metric_rows: List[dict] = []
    predictions = test[["run_id", GROUP_COLUMN, "repeat_index"] + FEATURE_COLUMNS].copy()
    predictions[f"actual_{target}"] = y_test

    for model_name, model in build_models().items():
        model.fit(x_train, y_train)
        predicted = np.asarray(model.predict(x_test), dtype=float)
        result = regression_metrics(y_test, predicted)
        metric_rows.append({"target": target, "model": model_name, **result})
        predictions[f"predicted_{target}_{model_name}"] = predicted
        trained_models[model_name] = model

    metric_rows.sort(key=lambda row: row["rmse"])
    return metric_rows, trained_models, predictions


def save_pickle(path: Path, value: object) -> None:
    with path.open("wb") as model_file:
        pickle.dump(value, model_file)


def print_metrics(target: str, metric_rows: Sequence[dict]) -> None:
    print(f"\n[{target}]")
    print(f"{'model':<22} {'MAE':>12} {'RMSE':>12} {'MAPE %':>12} {'R2':>12}")
    for row in metric_rows:
        print(
            f"{row['model']:<22} {row['mae']:>12.6f} {row['rmse']:>12.6f} "
            f"{row['mape_percent']:>12.3f} {row['r2']:>12.6f}"
        )


def train(dataset_path: Path, output_dir: Path) -> None:
    dataframe = load_dataset(dataset_path)
    train_rows, test_rows = split_by_geometry(dataframe)
    output_dir.mkdir(parents=True, exist_ok=True)

    train_groups = sorted(train_rows[GROUP_COLUMN].unique().tolist())
    test_groups = sorted(test_rows[GROUP_COLUMN].unique().tolist())
    print(
        f"Rows: train={len(train_rows)}, test={len(test_rows)}, total={len(dataframe)} | "
        f"Geometries: train={len(train_groups)}, test={len(test_groups)}"
    )
    print(f"Features: {FEATURE_COLUMNS}")

    all_metrics: List[dict] = []
    prediction_frames: List[pd.DataFrame] = []
    best_models: Dict[str, str] = {}

    for target in TARGET_COLUMNS:
        metric_rows, models, predictions = train_target(train_rows, test_rows, target)
        best_model_name = metric_rows[0]["model"]
        best_models[target] = best_model_name
        for model_name, model in models.items():
            save_pickle(output_dir / f"{target}_{model_name}.pkl", model)
        save_pickle(output_dir / f"best_{target}.pkl", models[best_model_name])
        all_metrics.extend(metric_rows)
        prediction_frames.append(predictions)
        print_metrics(target, metric_rows)
        print(f"Best model: {best_model_name} (lowest test RMSE)")

    merged_predictions = prediction_frames[0]
    prediction_keys = ["run_id", GROUP_COLUMN, "repeat_index"] + FEATURE_COLUMNS
    for frame in prediction_frames[1:]:
        new_columns = [column for column in frame.columns if column not in prediction_keys]
        merged_predictions = merged_predictions.merge(
            frame[prediction_keys + new_columns], on=prediction_keys, how="inner"
        )

    pd.DataFrame(all_metrics).to_csv(output_dir / "metrics.csv", index=False)
    merged_predictions.to_csv(output_dir / "test_predictions.csv", index=False)
    train_rows.to_csv(output_dir / "train_rows.csv", index=False)
    test_rows.to_csv(output_dir / "test_rows.csv", index=False)

    metadata = {
        "dataset": str(dataset_path),
        "features": FEATURE_COLUMNS,
        "targets": TARGET_COLUMNS,
        "group_column": GROUP_COLUMN,
        "test_size": TEST_SIZE,
        "random_seed": RANDOM_SEED,
        "row_counts": {
            "total": len(dataframe), "train": len(train_rows), "test": len(test_rows),
        },
        "geometry_counts": {
            "total": dataframe[GROUP_COLUMN].nunique(),
            "train": len(train_groups),
            "test": len(test_groups),
        },
        "train_geometry_ids": train_groups,
        "test_geometry_ids": test_groups,
        "best_models": best_models,
    }
    with (output_dir / "metadata.json").open("w") as metadata_file:
        json.dump(metadata, metadata_file, indent=2)

    print(f"\nSaved models and reports to {output_dir}")


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("dataset", nargs="?", type=Path, default=DATASET_PATH)
    parser.add_argument("--output-dir", type=Path, default=OUTPUT_DIR)
    return parser


def main(argv: Optional[Sequence[str]] = None) -> None:
    args = build_parser().parse_args(argv)
    train(args.dataset, args.output_dir)


if __name__ == "__main__":
    main()
