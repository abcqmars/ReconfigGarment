# MappingSolver

Segment mapping between two garments using a mixed-integer program (MIP) and turning-function polygon distance.

## Usage

```bash
pip install -r requirements.txt
python run_mapping_solver.py garment1_polygons.txt garment2_polygons.txt output_mapping.txt
```

## Input

Polygon export files in parafashion format (`Polygon` preferred; `ApproxPolygon` fallback).

## Output

- `output_mapping.txt` — segment index pairs with scores and rigid transforms
- `output_mapping_quads.txt` — fitted approx polygons for visualization

## Dependencies

Install packages from `requirements.txt`. This project also requires the external `turning_function` module on `PYTHONPATH`.

## Environment variables

| Variable | Default | Description |
|----------|---------|-------------|
| `PARAFASHION_QUAD_WORKERS` | auto | Parallel workers for polygon fitting |
| `PARAFASHION_QUAD_BACKEND` | `process` | `process`, `thread`, or `serial` |
| `PARAFASHION_APPROX_VERTICES` | `4,5` | Candidate vertex counts for approx fit |
