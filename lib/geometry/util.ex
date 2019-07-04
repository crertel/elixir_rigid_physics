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
end
