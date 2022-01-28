## Summary
MATLAB core provide the cubic splines functionality

## B-splines
An `order` `k` B-spline is formed by several pieces of polynomials of `degree`  `k-1`, with at most <img style="transform: translateY(0.1em); background: white;" src="https://render.githubusercontent.com/render/math?math=C%5E%7Bk-2%7D"> continuity at the breakpoints.

Given `n+1` control points <img style="transform: translateY(0.1em); background: white;" src="https://render.githubusercontent.com/render/math?math=%5Cmathbf%7Bp%7D_i">

A `knot` vector defines the set of non-descending breakpoints

<p align="center">
<img style="transform: translateY(0.1em); background: white;" src="https://render.githubusercontent.com/render/math?math=%5Cmathbf%7BT%7D%3D(t_0%2Ct_1%2C...%2Ct_m)">
</p>

<p align="center">
<img style="transform: translateY(0.1em); background: white;" src="https://render.githubusercontent.com/render/math?math=m%3Dn%2Bk">
</p>

`Nodes` <img style="transform: translateY(0.1em); background: white;" src="https://render.githubusercontent.com/render/math?math=%5Cxi_i"> are the averages of the knots

<p align="center">
<img style="transform: translateY(0.1em); background: white;" src="https://render.githubusercontent.com/render/math?math=%5Cxi_i%20%3D%20%5Cfrac%7B1%7D%7Bk-1%7D(t_%7Bi%2B1%7D%2Bt_%7Bi%2B2%7D%2B...%2Bt_%7Bi%2Bk-1%7D)">
</p>

### Code

`demo_spline.m`

```m
k = 4;  % degree = 3
n = 6;  % control points size 7
m = 10; % knots vector size 11

knots = [0 0 0 0 0.25 0.5 0.75 1 1 1 1];    % clamped uniform
```

- basis function size: `m+1-k`
<p align="center">
    <img width="380" src="img/bspline_basis.png">
</p>

- at each point, the sum of basis functions is equal to 1
<p align="center">
    <img width="380" src="img/bspline_basis_weight.png">
</p>

- Convex Hull: a span lies within the convex hull of the `k` control points that affect it.
<p align="center">
    <img width="380" src="img/bspline_convex_hull.png">
</p>

## Details

|function | details|
|--|--|
|`fnval`  |evaluate|
|`fnder`  |differentiate|
|`fndir`  |Directional derivative of function|
|`fnint`  |integrate|
|`fnmin`  |minimize|
|`fnzeros`|find zeros of|
|`fnplt`  |plot|
|`fnrfn`  |refine|
|`fnxtr`  |Extrapolate spline|
|`augknt` | providing boundary knots and also controlling the multiplicity of interior knots|
|`brk2knt`| supplying a knot sequence with specified multiplicities|
|`aptknt` | providing a knot sequence for a spline space of given order that is suitable for interpolation at given data sites|
|`optknt` | providing an optimal knot sequence for interpolation at given sites|
|`newknt` | a knot sequence perhaps more suitable for the function to be approximated|
|`aveknt` | Provide knot averages|


# Refs
- https://web.mit.edu/hyperbook/Patrikalakis-Maekawa-Cho/node15.html