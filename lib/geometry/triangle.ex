defmodule ElixirRigidPhysics.Geometry.Triangle do
  @moduledoc """
  Module for handling queries related to planar 3D triangles
  """
  alias Graphmath.Vec3

  require Record
  Record.defrecord(:triangle, a: {0.0, 0.0, 0.0}, b: {0.0, 1.0, 0.0}, c: {0.0, 0.0, 1.0})
  @type triangle :: record(:triangle, a: Vec3.vec3(), b: Vec3.vec3(), c: Vec3.vec3())

  require ElixirRigidPhysics.Geometry.Plane, as: Plane

  @doc """
  Creates a triangle given three points.

  ## Examples
    iex> # IO.puts "Test basic triangle creation from points"
    iex> require ElixirRigidPhysics.Geometry.Triangle, as: Triangle
    iex> a = {1.0, 0.0, 1.0}
    iex> b = {0.0, 0.0, 0.0}
    iex> c = {3.0, 0.0, 1.0}
    iex> Triangle.create_from_points( a, b, c )
    {:triangle, {1.0, 0.0, 1.0}, {0.0, 0.0, 0.0}, {3.0, 0.0, 1.0}}
  """
  @spec create_from_points(Vec3.vec3(), Vec3.vec3(), Vec3.vec3()) :: triangle
  def create_from_points(a, b, c) do
    triangle(a: a, b: b, c: c)
  end

  @doc """
  Creates plane from triangle.

  ## Examples
    iex> # IO.puts "Test plane creation from triangle"
    iex> require ElixirRigidPhysics.Geometry.Triangle, as: Triangle
    iex> a = {0.0, 0.0, 0.0}
    iex> b = {1.0, 0.0, 0.0}
    iex> c = {0.0, 0.0, 1.0}
    iex> t= Triangle.create_from_points( a, b, c )
    iex> Triangle.to_plane( t )
    {:plane, 0.0, -1.0, 0.0, 0.0}

    iex> # IO.puts "Test plane creation from scaled triangle"
    iex> require ElixirRigidPhysics.Geometry.Triangle, as: Triangle
    iex> a = {0.0, 0.0, 0.0}
    iex> b = {2.0, 0.0, 0.0}
    iex> c = {0.0, 0.0, 2.0}
    iex> t= Triangle.create_from_points( a, b, c )
    iex> Triangle.to_plane( t )
    {:plane, 0.0, -1.0, 0.0, 0.0}

    iex> # IO.puts "Test plane creation from flipped triangle"
    iex> require ElixirRigidPhysics.Geometry.Triangle, as: Triangle
    iex> a = {0.0, 0.0, 0.0}
    iex> b = {1.0, 0.0, 0.0}
    iex> c = {0.0, 0.0, 1.0}
    iex> t= Triangle.create_from_points( a, c, b  )
    iex> Triangle.to_plane( t )
    {:plane, 0.0, 1.0, 0.0, 0.0}

    iex> # IO.puts "Test plane creation from 3D triangle"
    iex> require ElixirRigidPhysics.Geometry.Triangle, as: Triangle
    iex> a = {0.0, 2.0, 0.0}
    iex> b = {1.0, 2.0, 0.0}
    iex> c = {0.0, 2.0, 1.0}
    iex> t= Triangle.create_from_points( a, b, c )
    iex> Triangle.to_plane( t )
    {:plane, 0.0, -1.0, 0.0, 2.0}

    iex> # IO.puts "Test plane creation from triangle"
    iex> require ElixirRigidPhysics.Geometry.Triangle, as: Triangle
    iex> a = {0.0, 0.0, 1.0}
    iex> b = {0.0, 1.0, 0.0}
    iex> c = {1.0, 0.0, 0.0}
    iex> sqrt_3_over_3 = :math.sqrt(3)/3.0
    iex> t= Triangle.create_from_points( a, b, c )
    iex> {:plane, pa, pb, pc, pd} = Triangle.to_plane( t )
    iex> Graphmath.Vec3.equal({pa,pb,pc},{-sqrt_3_over_3,-sqrt_3_over_3,-sqrt_3_over_3}, 0.000005)
    true
    iex> Float.round(pd - sqrt_3_over_3) == 0.0
    true
  """
  @spec to_plane(triangle) :: Plane.plane()
  def to_plane(triangle(a: a, b: b, c: c)) do
    ab = Vec3.subtract(b, a)
    ac = Vec3.subtract(c, a)
    n = ab
        |> Vec3.cross(ac)
        |> Vec3.normalize()
    Plane.create(n, a)
  end

  @doc """
  Converts a point in a triangle in barycentric coordinates into cartesian coordinates.

  ## Examples
    iex> # IO.puts "Check from_barycentric for a"
    iex> require ElixirRigidPhysics.Geometry.Triangle, as: Triangle
    iex> a = {0.0, 0.0, 1.0}
    iex> b = {0.0, 1.0, 0.0}
    iex> c = {1.0, 0.0, 0.0}
    iex> t= Triangle.create_from_points( a, b, c )
    iex> Triangle.from_barycentric(t, {1.0, 0.0, 0.0})
    {0.0, 0.0, 1.0}

    iex> # IO.puts "Check from_barycentric for b"
    iex> require ElixirRigidPhysics.Geometry.Triangle, as: Triangle
    iex> a = {0.0, 0.0, 1.0}
    iex> b = {0.0, 1.0, 0.0}
    iex> c = {1.0, 0.0, 0.0}
    iex> t= Triangle.create_from_points( a, b, c )
    iex> Triangle.from_barycentric(t, {0.0, 1.0, 0.0})
    {0.0, 1.0, 0.0}

    iex> # IO.puts "Check from_barycentric for a"
    iex> require ElixirRigidPhysics.Geometry.Triangle, as: Triangle
    iex> a = {0.0, 0.0, 1.0}
    iex> b = {0.0, 1.0, 0.0}
    iex> c = {1.0, 0.0, 0.0}
    iex> t= Triangle.create_from_points( a, b, c )
    iex> Triangle.from_barycentric(t, {0.0, 0.0, 1.0})
    {1.0, 0.0, 0.0}
  """
  @spec from_barycentric(triangle, {number, number, number}) :: Vec3.vec3()
  def from_barycentric(triangle(a: a, b: b, c: c), {u, v, w}) do
    u
    |> Vec3.weighted_sum(a, v, b)
    |> Vec3.add(Vec3.scale(c, w))
  end

  @doc """
  Gets the baryncetric coordinates of a point `q` in the space of a triangle `t`.

  Note that the point must be coplanar with the triangle for this to reliably make sense.

  ## Examples
    iex> # IO.puts "Check to_barycentric for a"
    iex> require ElixirRigidPhysics.Geometry.Triangle, as: Triangle
    iex> a = {0.0, 0.0, 1.0}
    iex> b = {0.0, 1.0, 0.0}
    iex> c = {1.0, 0.0, 0.0}
    iex> t= Triangle.create_from_points( a, b, c )
    iex> Triangle.to_barycentric(t, {0.0, 0.0, 1.0})
    {1.0, 0.0, 0.0}

    iex> # IO.puts "Check to_barycentric for b"
    iex> require ElixirRigidPhysics.Geometry.Triangle, as: Triangle
    iex> a = {0.0, 0.0, 1.0}
    iex> b = {0.0, 1.0, 0.0}
    iex> c = {1.0, 0.0, 0.0}
    iex> t= Triangle.create_from_points( a, b, c )
    iex> Triangle.to_barycentric(t, {0.0, 1.0, 0.0})
    {0.0, 1.0, 0.0}

    iex> # IO.puts "Check to_barycentric for a"
    iex> require ElixirRigidPhysics.Geometry.Triangle, as: Triangle
    iex> a = {0.0, 0.0, 1.0}
    iex> b = {0.0, 1.0, 0.0}
    iex> c = {1.0, 0.0, 0.0}
    iex> t= Triangle.create_from_points( a, b, c )
    iex> Triangle.to_barycentric(t, {1.0, 0.0, 0.0})
    {0.0, 0.0, 1.0}

    iex> # IO.puts "Check to_barycentric for center of abc"
    iex> require ElixirRigidPhysics.Geometry.Triangle, as: Triangle
    iex> a = {0.0, 0.0, 1.0}
    iex> b = {0.0, 1.0, 0.0}
    iex> c = {1.0, 0.0, 0.0}
    iex> t= Triangle.create_from_points( a, b, c )
    iex> Triangle.to_barycentric(t, {1/3, 1/3, 1/3})
    {1/3, 1/3, 1/3}
  """
  @spec to_barycentric(triangle, Vec3.vec3()) :: Vec3.vec3()
  def to_barycentric(triangle(a: a, b: b, c: c), q) do
    # note that a cross product has a magniture of twice the area of the tri formed by the vectors
    # see https://users.csc.calpoly.edu/~zwood/teaching/csc471/2017F/barycentric.pdf for derivation
    v_ba = Vec3.subtract(b, a)
    v_ca = Vec3.subtract(c, a)
    v_ac = Vec3.subtract(a, c)
    v_cb = Vec3.subtract(c, b)
    v_qb = Vec3.subtract(q, b)
    v_qc = Vec3.subtract(q, c)
    v_qa = Vec3.subtract(q, a)

    n = Vec3.cross(v_ba, v_ca)
    na = Vec3.cross(v_cb, v_qb)
    nb = Vec3.cross(v_ac, v_qc)
    nc = Vec3.cross(v_ba, v_qa)

    # minor trick using dot product to save the magnitude squared...change to formula (11) from above
    n_len_squared = Vec3.dot(n, n)

    {Vec3.dot(n, na) / n_len_squared, Vec3.dot(n, nb) / n_len_squared,
     Vec3.dot(n, nc) / n_len_squared}
  end
end
