defmodule ElixirRigidPhysics.Geometry.Util do
  @moduledoc """
  Module to handle oddball util functions for geometry stuff.
  """

  alias Graphmath.Vec3

  @doc """
  Function to get the closest point to a point `p` on a line segment spanning points `a` and `b`.

  Excellent derviation [here](https://math.stackexchange.com/a/2193733).

  ## Examples
    iex> # p coincident with a
    iex> ElixirRigidPhysics.Geometry.Util.closest_point_on_line_to_point( {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0},  {0.0, 1.0, 0.0})
    {0.0, 0.0, 0.0}

    iex> # p coincident with b
    iex> ElixirRigidPhysics.Geometry.Util.closest_point_on_line_to_point( {0.0, 1.0, 0.0}, {0.0, 0.0, 0.0},  {0.0, 1.0, 0.0})
    {0.0, 1.0, 0.0}

    iex> # p midway between a and b
    iex> ElixirRigidPhysics.Geometry.Util.closest_point_on_line_to_point( {0.0, 0.5, 0.0}, {0.0, 0.0, 0.0},  {0.0, 1.0, 0.0})
    {0.0, 0.5, 0.0}

    iex> # p closer to a
    iex> ElixirRigidPhysics.Geometry.Util.closest_point_on_line_to_point( {0.0, -0.5, 0.0}, {0.0, 0.0, 0.0},  {0.0, 1.0, 0.0})
    {0.0, 0.0, 0.0}

    iex> # p closer to b
    iex> ElixirRigidPhysics.Geometry.Util.closest_point_on_line_to_point( {0.0, 2.5, 0.0}, {0.0, 0.0, 0.0},  {0.0, 1.0, 0.0})
    {0.0, 1.0, 0.0}

    iex> # p far away from midpoint
    iex> ElixirRigidPhysics.Geometry.Util.closest_point_on_line_to_point( {1000.0, 0.5, 0.0}, {0.0, 0.0, 0.0},  {0.0, 1.0, 0.0})
    {0.0, 0.5, 0.0}
    iex> ElixirRigidPhysics.Geometry.Util.closest_point_on_line_to_point( {0.0, 0.5, 10000.0}, {0.0, 0.0, 0.0},  {0.0, 1.0, 0.0})
    {0.0, 0.5, 0.0}

  """
  @spec closest_point_on_line_to_point(Vec3.vec3(), Vec3.vec3(), Vec3.vec3()) :: Vec3.vec3()
  def closest_point_on_line_to_point(p, a, b) do
    v = Vec3.subtract(b, a)
    u = Vec3.subtract(a, p)
    dvu = Vec3.dot(v, u)

    t = -(dvu / Vec3.dot(v, v))

    if t >= 0.0 and t <= 1.0 do
      Vec3.add(Vec3.scale(a, 1 - t), Vec3.scale(b, t))
    else
      g0 = Vec3.length_squared(u)
      g1 = Vec3.length(v) + 2.0 * dvu + g0

      if g0 > g1 do
        b
      else
        a
      end
    end
  end

  @doc """
  Gets the distance and two closest points for a pair of line segments.

  Algorithm adapted from David Eberly's [c++ implementation](https://www.geometrictools.com/GTEngine/Include/Mathematics/GteDistSegmentSegment.h).

  You really probably want to read 10.8.2 in _Geometric Tools for Computer Graphics_ to have a chance in hell of understanding this.

  Even then, you'll still want to see Eberly's [writeup](https://www.geometrictools.com/Documentation/DistanceLine3Line3.pdf), since the algo here is different than the book.

  ## Examples
    iex> # check mutual degeneracy
    iex> alias ElixirRigidPhysics.Geometry.Util, as: GUtil
    iex> p = { {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}}
    iex> q = { {0.0, 2.0, 0.0}, {0.0, 2.0, 0.0}}
    iex> GUtil.nearest_points_for_segments( p, q )
    {2.0, {0.0, 0.0, 0.0}, {0.0, 2.0, 0.0}}

    iex> # check p degeneracy
    iex> alias ElixirRigidPhysics.Geometry.Util, as: GUtil
    iex> p = { {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}}
    iex> q = { {0.0, 2.0, 0.0}, {0.0, 4.0, 0.0}}
    iex> GUtil.nearest_points_for_segments( p, q )
    {2.0, {0.0, 0.0, 0.0}, {0.0, 2.0, 0.0}}

    iex> # check q degeneracy
    iex> alias ElixirRigidPhysics.Geometry.Util, as: GUtil
    iex> p = { {0.0, 0.0, 0.0}, {2.0, 0.0, 0.0}}
    iex> q = { {0.0, 2.0, 0.0}, {0.0, 2.0, 0.0}}
    iex> GUtil.nearest_points_for_segments( p, q )
    {2.0, {0.0, 0.0, 0.0}, {0.0, 2.0, 0.0}}

    iex> # check intersecting segments
    iex> alias ElixirRigidPhysics.Geometry.Util, as: GUtil
    iex> GUtil.nearest_points_for_segments( { {-1.0,0.0,0.0}, {1.0,0.0,0.0}}, {{0.0,-1.0,0.0},{0.0, 1.0, 0.0}})
    {0.0, {0.0,0.0,0.0}, {0.0, 0.0, 0.0}}

    iex> # check for corner intersection
    iex> alias ElixirRigidPhysics.Geometry.Util, as: GUtil
    iex> p = { {0.0, 0.0, 0.0}, {2.0, 0.0, 0.0}}
    iex> q = { {2.0, 0.0, 0.0}, {2.0, 2.0, 0.0}}
    iex> GUtil.nearest_points_for_segments( p, q )
    {0.0, {2.0, 0.0, 0.0}, {2.0, 0.0, 0.0}}

    iex> # check raised non-intersection
    iex> alias ElixirRigidPhysics.Geometry.Util, as: GUtil
    iex> GUtil.nearest_points_for_segments( { {-1.0,0.0,0.0}, {1.0,0.0,0.0}}, {{0.0,-1.0,1.0},{0.0, 1.0, 1.0}})
    {1.0, {0.0,0.0,0.0}, {0.0, 0.0, 1.0}}

    iex> # check collinear non-intersection
    iex> alias ElixirRigidPhysics.Geometry.Util, as: GUtil
    iex> p = { {1.0, 1.0, 1.0}, {3.0, 3.0, 3.0}}
    iex> q = { {-2.0, -2.0, -2.0}, {-5.0, -5.0, -5.0}}
    iex> sqrt_27 = :math.sqrt(27)
    iex> { sqrt_27, {1.0, 1.0, 1.0}, {-2.0, -2.0, -2.0}} == GUtil.nearest_points_for_segments( p, q )
    true

  """
  @spec nearest_points_for_segments({Vec3.t(), Vec3.t()}, {Vec3.t(), Vec3.t()}) ::
          {float, Vec3.t(), Vec3.t()}
  def nearest_points_for_segments({p0, p1}, {q0, q1}) do
    p1_to_p0 = Vec3.subtract(p1, p0)
    q1_to_q0 = Vec3.subtract(q1, q0)
    p0_to_q0 = Vec3.subtract(p0, q0)

    a = Vec3.dot(p1_to_p0, p1_to_p0)
    b = Vec3.dot(p1_to_p0, q1_to_q0)
    c = Vec3.dot(q1_to_q0, q1_to_q0)
    d = Vec3.dot(p1_to_p0, p0_to_q0)
    e = Vec3.dot(q1_to_q0, p0_to_q0)

    f00 = d
    f10 = f00 + a
    f01 = f00 - b
    f11 = f10 - b

    g00 = -e
    g10 = g00 - b
    g01 = g00 + c
    g11 = g10 + c

    # check for segment degeneracy
    {s, t} =
      cond do
        a > 0.0 and c > 0.0 ->
          # both segments are valid

          s_value_0 = get_clamped_root(a, f00, f10)
          s_value_1 = get_clamped_root(a, f01, f11)

          classify_0 =
            cond do
              s_value_0 <= 0.0 -> -1.0
              s_value_0 >= 1.0 -> 1.0
              true -> 0.0
            end

          classify_1 =
            cond do
              s_value_1 <= 0.0 -> -1.0
              s_value_1 >= 1.0 -> 1.0
              true -> 0.0
            end

          cond do
            classify_0 == -1.0 and classify_1 == -1.0 ->
              {0.0, get_clamped_root(c, g00, g01)}

            classify_0 == 1.0 and classify_1 == 1.0 ->
              {1.0, get_clamped_root(c, g10, g11)}

            true ->
              r_coeffs = {a, b, c, d, e}
              g = {g00, g01, g10, g11}

              {edges, ends} =
                compute_intersection(
                  {s_value_0, s_value_1},
                  {classify_0, classify_1},
                  b,
                  f00,
                  f10
                )

              compute_minimum_parameters(edges, ends, r_coeffs, g)
          end

        # q segment is degenerate
        a > 0.0 ->
          {get_clamped_root(a, f00, f10), 0.0}

        # p segment is degenerate
        c > 0.0 ->
          {0.0, get_clamped_root(c, g00, g01)}

        # both segments are degenerate!
        true ->
          {0.0, 0.0}
      end

    p_nearest = Vec3.lerp(p0, p1, s)
    q_nearest = Vec3.lerp(q0, q1, t)
    distance = p_nearest
              |> Vec3.subtract(q_nearest)
              |> Vec3.length()

    {distance, p_nearest, q_nearest}
  end

  @spec compute_intersection({number, number}, {number, number}, number, number, number) ::
          {{number, number}, {number, number, number, number}}
  defp compute_intersection({s_value_0, s_value_1}, {classify_0, classify_1}, b, f00, f10) do
    cond do
      classify_0 < 0.0 ->
        edge_0 = 0
        end_00 = 0.0
        end_01 = if b == 0.0, do: 0.5, else: f00 / b
        end_01 = if end_01 < 0.0 or end_01 > 1.0, do: 0.5, else: end_01

        {edge_1, end_10, end_11} =
          if classify_1 == 0 do
            edge_1 = 3
            end_10 = s_value_1
            end_11 = 1.0
            {edge_1, end_10, end_11}
          else
            edge_1 = 1
            end_10 = 1.0
            end_11 = if b == 0.0, do: 0.5, else: f10 / b
            end_11 = if end_11 < 0.0 or end_11 > 1.0, do: 0.5, else: end_11
            {edge_1, end_10, end_11}
          end

        {{edge_0, edge_1}, {end_00, end_01, end_10, end_11}}

      classify_0 == 0.0 ->
        edge_0 = 2
        end_00 = s_value_0
        end_01 = 0.0

        {edge_1, end_10, end_11} =
          cond do
            classify_1 < 0.0 ->
              edge_1 = 0
              end_10 = 0.0
              end_11 = if b == 0.0, do: 0.5, else: f00 / b
              end_11 = if end_11 < 0.0 or end_11 > 1.0, do: 0.5, else: end_11
              {edge_1, end_10, end_11}

            classify_1 == 0.0 ->
              {3, s_value_1, 1.0}

            true ->
              edge_1 = 1
              end_10 = 1.0
              end_11 = if b == 0.0, do: 0.5, else: f10 / b
              end_11 = if end_11 < 0.0 or end_11 > 1.0, do: 0.5, else: end_11
              {edge_1, end_10, end_11}
          end

        {{edge_0, edge_1}, {end_00, end_01, end_10, end_11}}

      true ->
        edge_0 = 1
        end_00 = 1.0
        end_01 = if b == 0.0, do: 0.5, else: f10 / b
        end_01 = if end_01 < 0.0 or end_01 > 1.0, do: 0.5, else: end_01

        {edge_1, end_10, end_11} =
          if classify_1 == 0.0 do
            {3, s_value_1, 1.0}
          else
            end_1 = 0
            end_10 = 0.0
            end_11 = if b == 0.0, do: 0.5, else: f00 / b
            end_11 = if end_11 < 0.0 or end_11 > 1.0, do: 0.5, else: end_11
            {end_1, end_10, end_11}
          end

        {{edge_0, edge_1}, {end_00, end_01, end_10, end_11}}
    end
  end

  @spec compute_minimum_parameters(
          {number, number},
          {number, number, number, number},
          {number, number, number, number, number},
          {number, number, number, number}
        ) :: {number, number}
  defp compute_minimum_parameters(
         {edge_0, edge_1},
         {end_00, end_01, end_10, end_11},
         {_a, b, c, _d, e},
         {g00, g01, g10, g11}
       ) do
    delta = end_11 - end_01
    h0 = delta * (-b * end_00 + c * end_01 - e)

    if h0 >= 0.0 do
      case edge_0 do
        0 -> {0.0, get_clamped_root(c, g00, g01)}
        1 -> {1.0, get_clamped_root(c, g10, g11)}
        _ -> {end_00, end_01}
      end
    else
      h1 = delta * (-b * end_10 + c * end_11 - e)

      if h1 <= 0.0 do
        case edge_1 do
          0 -> {0.0, get_clamped_root(c, g00, g01)}
          1 -> {1.0, get_clamped_root(c, g10, g11)}
          _ -> {end_10, end_11}
        end
      else
        z = min(max(h0 / (h0 - h1), 0.0), 1.0)
        omz = 1.0 - z
        {omz * end_00 + z * end_10, omz * end_01 + z * end_11}
      end
    end
  end

  @doc """
  Gets the root `z` of the linear function `h(z) = h(0) + sigma * z` on the interval [0,1], or the clamped root to the interval [0,1].
  Requires `h(0)` and `h(1) = h(0) + sigma` to have opposite signs (implying root on real line). Thanks Eberly! :)

  ## Examples
    iex> # test h0 >= 0
    iex> ElixirRigidPhysics.Geometry.Util.get_clamped_root(42, 1,  0)
    0.0

    iex> # test h1 <= 0
    iex> ElixirRigidPhysics.Geometry.Util.get_clamped_root(42, -1,  -3)
    1.0

    iex> # test sigma being 0
    iex> ElixirRigidPhysics.Geometry.Util.get_clamped_root(0.0, -1,  3)
    0.5

    iex> # test root over interval of [0,1]
    iex> ElixirRigidPhysics.Geometry.Util.get_clamped_root(0.05, -1,  3)
    0.5

    iex> # test normal behavior
    iex> ElixirRigidPhysics.Geometry.Util.get_clamped_root(1, -0.2,  1)
    0.2

  """
  @spec get_clamped_root(number(), number(), number()) :: number()
  def get_clamped_root(sigma, h0, h1) do
    cond do
      h0 >= 0.0 ->
        0.0

      h1 <= 0 ->
        1.0

      # add check because original C++ code -h0/0 -> infinity, but in Elixir would throw
      sigma == 0.0 ->
        0.5

      # Eberly suggests that this can be replaced with a bisection routine for `h(z)`, but that's slow
      true ->
        root = -h0 / sigma

        if root > 1 do
          0.5
        else
          root
        end
    end
  end
end
