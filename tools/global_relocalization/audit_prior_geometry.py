#!/usr/bin/env python3
"""Measure prior distortion in the established world frame; never fit or edit maps."""
import argparse
import hashlib
import json
from pathlib import Path

import numpy as np
from scipy.spatial import cKDTree
from sensor_map_batch import statistics


def read_xyz(path):
    with path.open("rb") as stream:
        header = []
        while True:
            line = stream.readline()
            if not line:
                raise ValueError("missing PCD data")
            header.append(line.decode().strip())
            if line.startswith(b"DATA "):
                break
        if "FIELDS x y z" not in header or "DATA binary" not in header or "SIZE 4 4 4" not in header:
            raise ValueError("expected binary float32 XYZ PCD")
        return np.frombuffer(stream.read(), dtype="<f4").reshape(-1, 3).copy()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--prior", type=Path, required=True)
    parser.add_argument("--reference", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    a = parser.parse_args()
    prior, reference = read_xyz(a.prior), read_xyz(a.reference)
    # Evaluate the left half used by localization, after the already frozen
    # one-pose frame convention. No ICP, trajectory fitting, or scale fitting.
    prior = prior[prior[:, 0] < 0]
    reference = reference[reference[:, 0] < 0]
    forward = cKDTree(reference).query(prior, workers=2)[0]
    backward = cKDTree(prior).query(reference, workers=2)[0]
    result = {"evaluation_only": True, "region": "x < 0", "map_registration_fitted": False,
              "maps_modified": False, "prior_points": len(prior), "reference_points": len(reference),
              "prior_sha256": hashlib.sha256(a.prior.read_bytes()).hexdigest(),
              "reference_sha256": hashlib.sha256(a.reference.read_bytes()).hexdigest(),
              "prior_to_reference_distance_m": statistics(forward),
              "reference_to_prior_distance_m": statistics(backward),
              "prior_fraction_within_0_1m": float(np.mean(forward <= .1)),
              "reference_fraction_covered_within_0_1m": float(np.mean(backward <= .1)),
              "interpretation": "Forward distance measures distortion plus sampling mismatch; reverse distance also includes unobserved map coverage. Neither is a registration accuracy score."}
    a.output.write_text(json.dumps(result, indent=2) + "\n")
    print(json.dumps(result, indent=2))


if __name__ == "__main__":
    main()
