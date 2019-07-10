defmodule ElixirRigidPhysics.Geometry.Plane do
  @moduledoc """
  Functions for creating and inteacting with [planes](https://en.wikipedia.org/wiki/Plane_(geometry)).

  Planes are all points `{x,y,z}` that fulfill the equation `ax + by + cz + d = 0`.

  We represent planes in the form: `{a,b,c,d}`, where `{a,b,c}` is is the normal of the plane, and `d` is `-dot(norm, point)`.
  This is known as [Hessian normal form](http://mathworld.wolfram.com/HessianNormalForm.html).
  """

  require Record
  Record.defrecord(:plane, a: 0.0, b: 0.0, c: 0.0, d: 0.0)
  @type plane :: record(:plane, a: number, b: number, c: number, d: number)

  alias Graphmath.Vec3

  @doc """
  Creates a plane given its normal and a point on the plane (for finding d).

  ## Examples
    iex> # test basic plane creation
    iex> require ElixirRigidPhysics.Geometry.Plane, as: Plane
    iex> Plane.create( {0.0, 1.0, 0.0}, {0.0, 0.0, 0.0})
    {:plane, 0.0, 1.0, 0.0, 0.0}

    iex> # test basic plane creation
    iex> require ElixirRigidPhysics.Geometry.Plane, as: Plane
    iex> Plane.create( {0.0, 1.0, 0.0}, {0.0, 1.0, 0.0})
    {:plane, 0.0, 1.0, 0.0, -1.0}
  """
  @spec create(Vec3.vec3(), Vec3.vec3()) :: plane
  def create({nx, ny, nz} = _normal, {px, py, pz} = _point) do
    d = -(nx * px + ny * py + nz * pz)
    plane(a: nx, b: ny, c: nz, d: d)
  end

  @verysmol 1.0e-12

  @doc """
  Checks the distance from a point to the plane. Returns positive values if in front of plane, negative behind,
  and zero if the point is coplanar.

  ## Examples
    iex> # Test in front
    iex> require ElixirRigidPhysics.Geometry.Plane, as: Plane
    iex> p = Plane.create( {0,1,0}, {0,0,0})
    iex> Plane.distance_to_point(p, {0,5,0})
    5.0

    iex> # Test behind
    iex> require ElixirRigidPhysics.Geometry.Plane, as: Plane
    iex> p = Plane.create( {1,0,0}, {0,0,0})
    iex> Plane.distance_to_point(p, {-3,5,3})
    -3.0

    iex> # Test coplanar
    iex> require ElixirRigidPhysics.Geometry.Plane, as: Plane
    iex> sqrt_third = :math.sqrt(1/3)
    iex> n = {sqrt_third, sqrt_third, sqrt_third}
    iex> p = Plane.create( n, {0,1,0})
    iex> t = Graphmath.Vec3.cross(n, {0,1,0})
    iex> Plane.distance_to_point(p, Graphmath.Vec3.add({0,1,0}, t))
    0.0

  """
  @spec distance_to_point(plane, Vec3.vec3()) :: float()
  def distance_to_point(plane(a: a, b: b, c: c, d: d) = _plane, {px, py, pz} = _point) do
    1.0 * (a * px + b * py + c * pz + d)
  end

  @doc """
  Projects a point on to a plane.

  ## Examples
    iex> # Project coplanar point
    iex> require ElixirRigidPhysics.Geometry.Plane, as: Plane
    iex> p = Plane.create( {0,1,0}, {0,4.0,0})
    iex> Plane.project_point_to_plane(p, {24.0,4.0,55.0})
    {24.0, 4.0, 55.0}

    iex> # Project front point
    iex> require ElixirRigidPhysics.Geometry.Plane, as: Plane
    iex> p = Plane.create( {0,0,1.0}, {0.0,0,3.0})
    iex> Plane.project_point_to_plane(p, {44.0,22.0, 43.0})
    {44.0, 22.0, 3.0}

    iex> # Project behind point
    iex> require ElixirRigidPhysics.Geometry.Plane, as: Plane
    iex> p = Plane.create( {0,0,1.0}, {0.0,0,3.0})
    iex> Plane.project_point_to_plane(p, {44.0,22.0, -43.0})
    {44.0, 22.0, 3.0}
  """
  def project_point_to_plane(plane(a: a, b: b, c: c, d: d) = _plane, {px, py, pz} = point) do
    distance = 1.0 * (a * px + b * py + c * pz + d)

    Vec3.scale({a, b, c}, -distance)
    |> Vec3.add(point)
  end

  @doc """
  Clips a point to exist in the positive half-space of a plane.

  ## Examples
    iex> # Project coplanar point
    iex> require ElixirRigidPhysics.Geometry.Plane, as: Plane
    iex> p = Plane.create( {0,0,1.0}, {0.0,0,3.0})
    iex> point =  {24.0,4.0,3.0}
    iex> Plane.clip_point(p, point)
    {24.0, 4.0, 3.0}

    iex> # Project front point
    iex> require ElixirRigidPhysics.Geometry.Plane, as: Plane
    iex> p = Plane.create( {0,0,1.0}, {0.0,0,3.0})
    iex> point = {24.0,4.0,55.0}
    iex> Plane.clip_point(p, point)
    {24.0, 4.0, 55.0}

    iex> # Project behind point
    iex> require ElixirRigidPhysics.Geometry.Plane, as: Plane
    iex> p = Plane.create( {0,0,1.0}, {0.0,0,3.0})
    iex> point = {24.0,4.0,-55.0}
    iex> Plane.clip_point(p, point)
    {24.0, 4.0, 3.0}
  """
  def clip_point(plane(a: a, b: b, c: c, d: d), {px, py, pz} = point) do
    distance = 1.0 * (a * px + b * py + c * pz + d)

    if distance >= @verysmol do
      point
    else
      # we're behind it, must project onto plane.
      Vec3.scale({a, b, c}, -distance)
      |> Vec3.add(point)
    end
  end
end
