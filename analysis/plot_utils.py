#!/usr/bin/env python3
import os
from typing import Iterable, List, Optional, Sequence, Tuple, Union

import numpy as np
import matplotlib.pyplot as plt

def _ensure_parent_dir(outpath: str) -> None:
    parent = os.path.dirname(os.path.abspath(outpath))
    if parent:
        os.makedirs(parent, exist_ok=True)

def save_simple_xy_plot(
        x: Union[Sequence[float], np.ndarray],
        y: Union[Sequence[float], np.ndarray],
        *,
        xlabel: str,
        ylabel: str,
        outpath: str,
        title: Optional[str] = None,
        kind: str = "line", # "line" or "scatter"
        label: Optional[str] = None,
        grid: bool = True,
        xscale: str= "linear",
        yscale: str= "linear",
    ) -> None:

    x_arr = np.asarray(x, dtype=float)
    y_arr = np.asarray(y, dtype=float)
    if x_arr.shape != y_arr.shape:
        raise ValueError(f"x and y must have the same shape, got {x_arr.shape} vs {y_arr.shape}")
    if x_arr.size == 0:
        raise ValueError("x and y are empty")

    _ensure_parent_dir(outpath)

    plt.figure()
    if kind == "line":
        plt.plot(x_arr, y_arr, label=label)
    elif kind == "scatter":
        plt.scatter(x_arr, y_arr, label=label)
    else:
        raise ValueError(f"unknown kind={kind!r}, expected 'line' or 'scatter'")

    plt.xlabel(xlabel)
    plt.ylabel(ylabel)
    if title is not None and title != "":
        plt.title(title)
    plt.xscale(xscale)
    plt.yscale(yscale)
    if grid:
        plt.grid(True)
    if label is not None:
        plt.legend()

    plt.savefig(outpath, bbox_inches="tight")
    plt.close()

def save_simple_xy_multi_scatter(
        series: List[Tuple[str, Union[Sequence[float], np.ndarray], Union[Sequence[float], np.ndarray]]],
        *,
        xlabel: str,
        ylabel: str,
        outpath: str,
        title: Optional[str] = None,
        grid: bool = True,
        xscale: str = "linear",
        yscale: str = "linear",
    ) -> None:

    if len(series) == 0:
        raise ValueError("series is empty")

    _ensure_parent_dir(outpath)
    plt.figure()

    any_plotted = False
    for label, x, y in series:
        x_arr = np.asarray(x, dtype=float)
        y_arr = np.asarray(y, dtype=float)
        if x_arr.shape != y_arr.shape:
            raise ValueError(f"series '{label}': x and y must have same shape")
        if x_arr.size == 0:
            continue
        plt.scatter(x_arr,y_arr, label=label)
        any_plotted = True

    if not any_plotted:
        raise ValueError("all series wer empty")

    plt.xlabel(xlabel)
    plt.ylabel(ylabel)
    if title is not None and title != "":
        plt.title(title)
    plt.xscale(xscale)
    plt.yscale(yscale)
    if grid:
        plt.grid(True)
    if label is not None:
        plt.legend()

    plt.savefig(outpath, bbox_inches="tight")
    plt.close()
        

        
