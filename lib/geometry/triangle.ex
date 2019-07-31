defmodule ElixirRigidPhysics.Geometry.Triangle do
  @moduledoc """
  Module for handling queries related to planar 3D triangles
  """
  alias Graphmath.Vec3

  require Record
  Record.defrecord(:triangle, a: {0.0, 0.0, 0.0}, b: {0.0, 1.0, 0.0}, c: {0.0, 0.0, 1.0})
  @type triangle :: record(:triangle, a: Vec3.vec3, b: Vec3.vec3, c: Vec3.vec3)

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
  @spec create_from_points(Vec3.vec3, Vec3.vec3, Vec3.vec3) :: triangle
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
    n = Vec3.cross(ab, ac) |> Vec3.normalize()
    Plane.create(n, a)
  end
end
